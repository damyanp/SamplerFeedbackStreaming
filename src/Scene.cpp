//*********************************************************
//
// Copyright 2020 Intel Corporation 
//
// Permission is hereby granted, free of charge, to any 
// person obtaining a copy of this software and associated 
// documentation files(the "Software"), to deal in the Software 
// without restriction, including without limitation the rights 
// to use, copy, modify, merge, publish, distribute, sublicense, 
// and/or sell copies of the Software, and to permit persons to 
// whom the Software is furnished to do so, subject to the 
// following conditions :
// The above copyright notice and this permission notice shall 
// be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
// DEALINGS IN THE SOFTWARE.
//
//*********************************************************

#include "pch.h"

#include "Scene.h"

#include "D3D12GpuTimer.h"

#include "Gui.h"
#include "TextureViewer.h"
#include "BufferViewer.h"
#include "FrustumViewer.h"

#include "WindowCapture.h"

#pragma comment(lib, "d3d12.lib")
#pragma comment(lib, "d3dcompiler.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "dxguid.lib")

#pragma comment(lib, "TileUpdateManager.lib")

using namespace DirectX;

// NOTE: the last value must be 0 for TSS. It signifies the pixel has been written to
const FLOAT Scene::m_clearColor[4] = { 0, 0, 0.05f, 0 };

enum class DescriptorHeapOffsets
{
    FRAME_CBV,         // b0
    GUI,
    SHARED_MIN_MIP_MAP,

    NumEntries
};

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
Scene::Scene(const CommandLineArgs& in_args, HWND in_hwnd) :
    m_args(in_args)
    , m_hwnd(in_hwnd)
    , m_fullScreen(false)
    , m_windowInfo{}
    , m_windowedSupportsTearing(false)
    , m_deviceRemoved(false)
    , m_frameIndex(0)
    , m_renderFenceValue(0)
    , m_frameFenceValues{}
    , m_renderFenceEvent(0)
    , m_rtvDescriptorSize(0)
    , m_srvUavCbvDescriptorSize(0)
    , m_dsvDescriptorSize(0)
    , m_aspectRatio(0)
    , m_pFrameConstantData(nullptr)

    // visuals
    , m_showFrustum(!in_args.m_visualizeFrustum) // need to force first-time creation
    , m_useDirectStorage(in_args.m_useDirectStorage)
    , m_pGui(nullptr)
    , m_pTextureViewer(nullptr)
    , m_pMinMipMapViewer(nullptr)
    , m_pFeedbackViewer(nullptr)
    , m_pFrustumViewer(nullptr)

    // thread
    , m_queueFeedbackIndex(0)
    , m_prevNumFeedbackObjects(SharedConstants::SWAP_CHAIN_BUFFER_COUNT, 1)

    // statistics
    , m_renderThreadTimes(in_args.m_statisticsNumFrames)
    , m_updateFeedbackTimes(in_args.m_statisticsNumFrames)
    , m_pGpuTimer(nullptr)
{
    m_windowInfo.cbSize = sizeof(WINDOWINFO);
#if ENABLE_DEBUG_LAYER
    InitDebugLayer();
#endif

    UINT flags = 0;
#ifdef _DEBUG
    flags |= DXGI_CREATE_FACTORY_DEBUG;
#endif
    if (FAILED(CreateDXGIFactory2(flags, IID_PPV_ARGS(&m_factory))))
    {
        flags &= ~DXGI_CREATE_FACTORY_DEBUG;
        ThrowIfFailed(CreateDXGIFactory2(flags, IID_PPV_ARGS(&m_factory)));
    }

    ComPtr<IDXGIAdapter1> adapter;
    if (m_args.m_adapterDescription.size())
    {
        std::wstring wideAdapterDesc = m_args.m_adapterDescription;
        for (auto& c : wideAdapterDesc) { c = ::towlower(c); }

        for (UINT i = 0; m_factory->EnumAdapters1(i, &adapter) != DXGI_ERROR_NOT_FOUND; ++i)
        {
            DXGI_ADAPTER_DESC1 desc;
            ThrowIfFailed(adapter->GetDesc1(&desc));
            std::wstring description(desc.Description);
            for (auto& c : description) { c = ::towlower(c); }
            std::size_t found = description.find(wideAdapterDesc);
            if (found != std::string::npos)
            {
                break;
            }
        }
    }
    ThrowIfFailed(D3D12CreateDevice(adapter.Get(), D3D_FEATURE_LEVEL_12_0, IID_PPV_ARGS(&m_device)));

    // does this device support sampler feedback?
    D3D12_FEATURE_DATA_D3D12_OPTIONS7 feedbackOptions{};
    m_device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS7, &feedbackOptions, sizeof(feedbackOptions));

    if (0 == feedbackOptions.SamplerFeedbackTier)
    {
        MessageBox(0, L"Sampler Feedback not supported", L"Error", MB_OK);
    }

    D3D12_FEATURE_DATA_D3D12_OPTIONS tileOptions{};
    m_device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS, &tileOptions, sizeof(tileOptions));

    if (0 == tileOptions.TiledResourcesTier)
    {
        MessageBox(0, L"Tiled Resources not supported", L"Error", MB_OK);
    }

    m_pGpuTimer = new D3D12GpuTimer(m_device.Get(), 8, D3D12GpuTimer::TimerType::Direct);

    // get the adapter this device was created with
    LUID adapterLUID = m_device->GetAdapterLuid();
    ThrowIfFailed(m_factory->EnumAdapterByLuid(adapterLUID, IID_PPV_ARGS(&m_adapter)));

    // descriptor sizes
    m_rtvDescriptorSize = m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
    m_srvUavCbvDescriptorSize = m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
    m_dsvDescriptorSize = m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_DSV);

    // creation order below matters
    CreateDescriptorHeaps();
    CreateCommandQueue();
    CreateSwapChain();
    CreateFence();
    
    StartStreamingLibrary();

    CreateSampler();
    CreateConstantBuffers();

    float eyePos = 100.0f;
    XMVECTOR vEyePt = XMVectorSet(eyePos, eyePos, eyePos, 1.0f);
    XMVECTOR lookAt = XMVectorSet(0.0f, 0.0f, 0.0f, 1.0f);
    XMVECTOR vUpVec = XMVectorSet(0.0f, 1.0f, 0.0f, 1.0f);

    m_viewMatrix = XMMatrixLookAtLH(vEyePt, lookAt, vUpVec);
    XMVECTOR pDet;
    m_viewMatrixInverse = XMMatrixInverse(&pDet, m_viewMatrix);

    m_pGui = new Gui(m_hwnd, m_device.Get(), m_srvHeap.Get(), (UINT)DescriptorHeapOffsets::GUI, m_swapBufferCount, SharedConstants::SWAP_CHAIN_FORMAT, m_args);

    m_pFrustumViewer = new FrustumViewer(m_device.Get(),
        SharedConstants::SWAP_CHAIN_FORMAT,
        SharedConstants::DEPTH_FORMAT,
        m_args.m_sampleCount,
        [&](ID3D12Resource* out_pBuffer, const void* in_pBytes, size_t in_numBytes, D3D12_RESOURCE_STATES in_finalState)
        {
            SceneObjects::InitializeBuffer(out_pBuffer, in_pBytes, in_numBytes, in_finalState);
        });

    // statistics gathering
    if (m_args.m_timingFrameFileName.size() && (m_args.m_timingStopFrame >= m_args.m_timingStartFrame))
    {
        m_csvFile = std::make_unique<FrameEventTracing>(m_args);
    }
}

Scene::~Scene()
{
    WaitForGpu();
    m_pTileUpdateManager->Finish();

    if (GetSystemMetrics(SM_REMOTESESSION) == 0)
    {
        m_swapChain->SetFullscreenState(FALSE, nullptr);
    }

    CloseHandle(m_renderFenceEvent);
    m_frameConstantBuffer->Unmap(0, nullptr);

    delete m_pGpuTimer;
    delete m_pGui;
    
    DeleteTerrainViewers();

    delete m_pFrustumViewer;

    for (auto o : m_objects)
    {
        delete o;
    }

    for (auto h : m_sharedHeaps)
    {
        delete h;
    }
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
RECT Scene::GetGuiRect()
{
    RECT r{};
    r.right = (LONG)m_pGui->GetWidth();
    r.bottom = (LONG)m_pGui->GetHeight();

    return r;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void Scene::MoveView(int in_x, int in_y, int in_z)
{
    float translationRate = 0.1f * GetFrameTime();

    if (0x8000 & GetKeyState(VK_SHIFT))
    {
        translationRate *= 8;
    }

    float x = in_x * translationRate;
    float y = in_y * translationRate;
    float z = in_z * -translationRate;
    XMMATRIX translation = XMMatrixTranslation(x, y, z);

    m_viewMatrix = XMMatrixMultiply(m_viewMatrix, translation);
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void Scene::RotateView(float in_x, float in_y, float in_z)
{
    XMMATRIX rotation = XMMatrixRotationRollPitchYaw(in_x, 0, in_z);

    if (in_y)
    {
        // NOTE: locking the "up" axis feels great when navigating the terrain
        // however, it breaks the controls when flying to other planets
        XMMATRIX rotY = XMMatrixIdentity();
        if (m_args.m_cameraUpLock)
        {
            // this prevents spin while panning the terrain, but breaks if the user intentionally rotates in Z
            rotY = XMMatrixRotationAxis(XMVectorSet(0, 1, 0, 1), in_y);
        }
        else
        {
            // this rotates correctly with any z axis rotation, but "up" can drift:
            XMVECTOR yAxis = m_viewMatrixInverse.r[1];
            rotY = XMMatrixRotationNormal(yAxis, in_y);
        }

        XMVECTOR xLate = XMVectorSetW(m_viewMatrixInverse.r[3], 0);
        rotY = XMMatrixMultiply(XMMatrixTranslationFromVector(-xLate), rotY);
        rotY = XMMatrixMultiply(rotY, XMMatrixTranslationFromVector(xLate));

        m_viewMatrix = XMMatrixMultiply(rotY, m_viewMatrix);
    }

    m_viewMatrix = XMMatrixMultiply(m_viewMatrix, rotation);

    XMVECTOR pDet;
    m_viewMatrixInverse = XMMatrixInverse(&pDet, m_viewMatrix);
}


void Scene::RotateViewKey(int in_x, int in_y, int in_z)
{
    float rotationRate = 0.001f * GetFrameTime();
    float x = in_x * -rotationRate;
    float y = in_y * rotationRate;
    float z = in_z * -rotationRate;
    RotateView(x, y, z);
}

void Scene::RotateViewPixels(int in_x, int in_y)
{
    float xRadians = (sin(m_fieldOfView) / m_viewport.Width) * 2.0f;
    float x = float(in_x) * xRadians;
    float y = float(in_y) * xRadians;
    RotateView(x, y, 0);
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
float Scene::GetFrameTime()
{
    return 1000.0f * m_renderThreadTimes.GetAverageTotal();
}

//-----------------------------------------------------------------------------
// common behavior for device removed/reset
//-----------------------------------------------------------------------------
bool Scene::IsDeviceOk(HRESULT in_hr)
{
    bool success = true;
    if ((DXGI_ERROR_DEVICE_REMOVED == in_hr) || (DXGI_ERROR_DEVICE_RESET == in_hr))
    {
        //HRESULT hr = m_device->GetDeviceRemovedReason();
        m_deviceRemoved = true;
        success = false;
    }
    else
    {
        ThrowIfFailed(in_hr);
    }
    return success;
}

//-----------------------------------------------------------------------------
// handle in/out of fullscreen immediately. defer render target size changes
// FIXME: 1st transition to full-screen on multi-gpu, app disappears (?) - hit ESC and try again
// FIXME: full-screen does not choose the nearest display for the associated adapter, it chooses the 1st
//-----------------------------------------------------------------------------
void Scene::Resize(bool in_fullScreen)
{
    // do not re-enter resize while handling a resize
    if (m_resetSwapChain) return;

    // prevent re-entry here until after we've gotten all the way through
    m_resetSwapChain = true;

    // remember placement for restore from full screen
    static WINDOWPLACEMENT windowPlacement;

    if (m_fullScreen != in_fullScreen)
    {
        WaitForGpu();

        // can't full screen with remote desktop
        bool canFullScreen = (GetSystemMetrics(SM_REMOTESESSION) == 0);

        if (in_fullScreen)
        {
            // remember the current placement so we can restore via vk_esc
            GetWindowPlacement(m_hwnd, &windowPlacement);

            // take the first attached monitor
            // FIXME? could search for the nearest monitor.
            MONITORINFO monitorInfo;
            monitorInfo.cbSize = sizeof(monitorInfo);
            GetMonitorInfo(MonitorFromWindow(m_hwnd, MONITOR_DEFAULTTONEAREST), &monitorInfo);

            ComPtr<IDXGIOutput> dxgiOutput;
            HRESULT result = m_adapter->EnumOutputs(0, &dxgiOutput);
            if (SUCCEEDED(result))
            {
                ThrowIfFailed(m_swapChain->SetFullscreenState(true, dxgiOutput.Get()));
            }
            else // enumerate may fail when multi-gpu and cloning displays
            {
                canFullScreen = false;
                auto width = ::GetSystemMetrics(SM_CXSCREEN);
                auto height = ::GetSystemMetrics(SM_CYSCREEN);
                SetWindowPos(m_hwnd, NULL, 0, 0, width, height, SWP_NOZORDER | SWP_NOACTIVATE);
            }
        }
        else
        {
            // when leaving full screen, the previous window state isn't restored by the OS
            // however, we saved it earlier...
            ThrowIfFailed(m_swapChain->SetFullscreenState(false, nullptr));
            int left = windowPlacement.rcNormalPosition.left;
            int top = windowPlacement.rcNormalPosition.top;
            int width = windowPlacement.rcNormalPosition.right - left;
            int height = windowPlacement.rcNormalPosition.bottom - top;
            SetWindowPos(m_hwnd, NULL, left, top, width, height, SWP_SHOWWINDOW);
        }
    }
    DoResize();
    // at this point, full screen state has (theoretically) been changed if required
    m_fullScreen = in_fullScreen;

    m_resetSwapChain = false;
}

//-----------------------------------------------------------------------------
// handle resize of render targets
//-----------------------------------------------------------------------------
void Scene::DoResize()
{
    // make sure we are paused at this point
    WaitForGpu();

    RECT rect{};
    GetClientRect(m_hwnd, &rect);
    UINT width = rect.right - rect.left;
    UINT height = rect.bottom - rect.top;

    // ignore resize events for 0-sized window
    if ((0 == height) || (0 == width))
    {
        // not a fatal error. just ignore it.
        return;
    }

    m_viewport = CD3DX12_VIEWPORT(0.0f, 0.0f, static_cast<FLOAT>(width), static_cast<FLOAT>(height));
    m_scissorRect = CD3DX12_RECT(0, 0, width, height);
    m_aspectRatio = m_viewport.Width / m_viewport.Height;

    float nearZ = 1.0f;
    float farZ = 100000.0f;
    m_projection = DirectX::XMMatrixPerspectiveFovLH(m_fieldOfView, m_aspectRatio, nearZ, farZ);

    // release references
    for (UINT i = 0; i < m_swapBufferCount; i++)
    {
        m_renderTargets[i].Reset();
    }

    UINT flags = 0;
    if (m_windowedSupportsTearing)
    {
        flags |= DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING;
    }
    HRESULT hr = m_swapChain->ResizeBuffers(0, width, height, DXGI_FORMAT_UNKNOWN, flags);
    bool success = IsDeviceOk(hr);

    if (success)
    {
        // Create a RTV for each frame.
        CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(m_rtvHeap->GetCPUDescriptorHandleForHeapStart());
        for (UINT i = 0; i < m_swapBufferCount; i++)
        {
            ThrowIfFailed(m_swapChain->GetBuffer(i, IID_PPV_ARGS(&m_renderTargets[i])));

            std::wstringstream w;
            w << "m_renderTargets[" << i << "]";
            m_renderTargets[i]->SetName(w.str().c_str());

            m_device->CreateRenderTargetView(m_renderTargets[i].Get(), nullptr, rtvHandle);
            rtvHandle.Offset(1, m_rtvDescriptorSize);
        }

        CreateRenderTargets();

        m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();

        // UI uses window dimensions
        m_args.m_windowWidth = width;
        m_args.m_windowHeight = height;
    }
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void Scene::CreateDescriptorHeaps()
{
    // shader resource view (SRV) heap for e.g. textures
    D3D12_DESCRIPTOR_HEAP_DESC srvHeapDesc = {};
    srvHeapDesc.NumDescriptors = (UINT)DescriptorHeapOffsets::NumEntries +
        (m_args.m_maxNumObjects * (UINT)SceneObjects::Descriptors::NumEntries);
    srvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    srvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    ThrowIfFailed(m_device->CreateDescriptorHeap(&srvHeapDesc, IID_PPV_ARGS(&m_srvHeap)));
    NAME_D3D12_OBJECT(m_srvHeap);

    // render target view heap
    // NOTE: we have an MSAA target plus a swap chain, so m_swapBufferCount + 1
    D3D12_DESCRIPTOR_HEAP_DESC rtvHeapDesc = {};
    rtvHeapDesc.NumDescriptors = m_swapBufferCount + 1;
    rtvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
    ThrowIfFailed(m_device->CreateDescriptorHeap(&rtvHeapDesc, IID_PPV_ARGS(&m_rtvHeap)));

    // depth buffer view heap
    D3D12_DESCRIPTOR_HEAP_DESC dsvHeapDesc = {};
    dsvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
    dsvHeapDesc.NumDescriptors = 1;
    ThrowIfFailed(m_device->CreateDescriptorHeap(&dsvHeapDesc, IID_PPV_ARGS(&m_dsvHeap)));
    NAME_D3D12_OBJECT(m_dsvHeap);
}

//-----------------------------------------------------------------------------
// Create synchronization objects and wait until assets have been uploaded to the GPU.
//-----------------------------------------------------------------------------
void Scene::CreateFence()
{
    ThrowIfFailed(m_device->CreateFence(
        m_renderFenceValue,
        D3D12_FENCE_FLAG_NONE,
        IID_PPV_ARGS(&m_renderFence)));

    // Create an event handle to use for frame synchronization.
    m_renderFenceEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr);
    if (m_renderFenceEvent == nullptr)
    {
        ThrowIfFailed(HRESULT_FROM_WIN32(GetLastError()));
    }
}

//-----------------------------------------------------------------------------
// creates queue, direct command list, and command allocators
//-----------------------------------------------------------------------------
void Scene::CreateCommandQueue()
{
    // Describe and create the command queue.
    D3D12_COMMAND_QUEUE_DESC queueDesc = {};
    queueDesc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
    queueDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
    ThrowIfFailed(m_device->CreateCommandQueue(&queueDesc, IID_PPV_ARGS(&m_commandQueue)));
    m_commandQueue->SetName(L"m_commandQueue");

    for (UINT i = 0; i < m_swapBufferCount; i++)
    {
        ThrowIfFailed(m_device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&m_commandAllocators[i])));

        std::wstringstream cmdAllocName;
        cmdAllocName << "m_commandAllocators #" << i;
        m_commandAllocators[i]->SetName(cmdAllocName.str().c_str());
    }

    m_device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, m_commandAllocators[0].Get(), nullptr, IID_PPV_ARGS(&m_commandList));
    m_commandList->SetName(L"m_commandList");
    m_commandList->Close();
}

//-----------------------------------------------------------------------------
// note creating the swap chain requires a command queue
// hence, if the command queue changes, we must re-create the swap chain
// command queue can change if we toggle the Intel command queue extension
//-----------------------------------------------------------------------------
void Scene::CreateSwapChain()
{
    // tearing supported for full-screen borderless windows?
    if (m_fullScreen)
    {
        m_windowedSupportsTearing = false;
    }
    else
    {
        BOOL allowTearing = FALSE;
        const HRESULT result = m_factory->CheckFeatureSupport(DXGI_FEATURE_PRESENT_ALLOW_TEARING, &allowTearing, sizeof(allowTearing));
        m_windowedSupportsTearing = SUCCEEDED(result) && allowTearing;
    }

    GetWindowInfo(m_hwnd, &m_windowInfo);

    // Describe and create the swap chain.
    DXGI_SWAP_CHAIN_DESC1 swapChainDesc = {};
    swapChainDesc.BufferCount = SharedConstants::SWAP_CHAIN_BUFFER_COUNT;
    swapChainDesc.Width = m_windowInfo.rcClient.right - m_windowInfo.rcClient.left;
    swapChainDesc.Height = m_windowInfo.rcClient.bottom - m_windowInfo.rcClient.top;
    swapChainDesc.Format = SharedConstants::SWAP_CHAIN_FORMAT;
    swapChainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    swapChainDesc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
    swapChainDesc.SampleDesc.Count = 1;
    swapChainDesc.Flags = 0;

    if (m_windowedSupportsTearing)
    {
        swapChainDesc.Flags |= DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING;
    }

    DXGI_SWAP_CHAIN_FULLSCREEN_DESC* pFullScreenDesc = nullptr;

    // if full screen mode, launch into the current settings
    DXGI_SWAP_CHAIN_FULLSCREEN_DESC fullScreenDesc = {};
    IDXGIOutput* pOutput = nullptr;

    // on switch to full screen, try to move to a monitor attached to the adapter
    // if no monitor attached, just use the "current" display
    if (m_fullScreen)
    {
        // get the dimensions of the primary monitor, same as GetDeviceCaps( hdcPrimaryMonitor, HORZRES)
        swapChainDesc.Width = GetSystemMetrics(SM_CXSCREEN);
        swapChainDesc.Height = GetSystemMetrics(SM_CYSCREEN);
        // primary monitor has 0,0 as top-left
        UINT left = 0;
        UINT top = 0;

        // take the first attached monitor
        m_adapter->EnumOutputs(0, &pOutput);
        if (pOutput)
        {
            DXGI_OUTPUT_DESC outputDesc;
            pOutput->GetDesc(&outputDesc);
            swapChainDesc.Width = outputDesc.DesktopCoordinates.right - outputDesc.DesktopCoordinates.left;
            swapChainDesc.Height = outputDesc.DesktopCoordinates.bottom - outputDesc.DesktopCoordinates.top;
            left = outputDesc.DesktopCoordinates.left;
            top = outputDesc.DesktopCoordinates.top;
        }
        SetWindowLongPtr(m_hwnd, GWL_STYLE, WS_VISIBLE | WS_POPUP);
        SetWindowPos(m_hwnd, HWND_TOP, left, top, swapChainDesc.Width, swapChainDesc.Height,
            SWP_FRAMECHANGED);

        fullScreenDesc.Windowed = FALSE;
        pFullScreenDesc = &fullScreenDesc;
    }

    ComPtr<IDXGISwapChain1> swapChain;
    ThrowIfFailed(m_factory->CreateSwapChainForHwnd(m_commandQueue.Get(), m_hwnd,
        &swapChainDesc, pFullScreenDesc, pOutput, &swapChain));

    /*
    want full screen with tearing.
    from MSDN, DXGI_PRESENT_ALLOW_TEARING:
    - The swap chain must be created with the DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING flag.
    - It can only be used in windowed mode.
    - To use this flag in full screen Win32 apps, the application should present to a fullscreen borderless window
    and disable automatic ALT+ENTER fullscreen switching using IDXGIFactory::MakeWindowAssociation.
     */
    ThrowIfFailed(m_factory->MakeWindowAssociation(m_hwnd,
        DXGI_MWA_NO_WINDOW_CHANGES | DXGI_MWA_NO_ALT_ENTER | DXGI_MWA_NO_PRINT_SCREEN));

    ThrowIfFailed(swapChain.As(&m_swapChain));

    if ((GetSystemMetrics(SM_REMOTESESSION) == 0) && (m_fullScreen))
    {
        m_swapChain->SetFullscreenState(TRUE, nullptr);
    }

    m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();
}

//-----------------------------------------------------------------------------
// Enable the D3D12 debug layer.
//-----------------------------------------------------------------------------
void Scene::InitDebugLayer()
{
    OutputDebugString(L"<<< WARNING: DEBUG LAYER ENABLED >>>\n");
    {
        ID3D12Debug1* pDebugController = nullptr;
        if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&pDebugController))))
        {
            //pDebugController->SetEnableGPUBasedValidation(TRUE);
            pDebugController->EnableDebugLayer();
            pDebugController->Release();
        }
    }
}

//-----------------------------------------------------------------------------
// modified next-frame logic to return a handle if a wait is required.
// NOTE: be sure to check for non-null handle before WaitForSingleObjectEx() (or equivalent)
//-----------------------------------------------------------------------------
void Scene::MoveToNextFrame()
{
    // Assign the current fence value to the current frame.
    m_frameFenceValues[m_frameIndex] = m_renderFenceValue;

    // Signal and increment the fence value.
    ThrowIfFailed(m_commandQueue->Signal(m_renderFence.Get(), m_renderFenceValue));
    m_renderFenceValue++;

    // Update the frame index.
    m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();

    // If the next frame is not ready to be rendered yet, wait until it is ready.
    if (m_renderFence->GetCompletedValue() < m_frameFenceValues[m_frameIndex])
    {
        ThrowIfFailed(m_renderFence->SetEventOnCompletion(m_frameFenceValues[m_frameIndex], m_renderFenceEvent));
        WaitForSingleObject(m_renderFenceEvent, INFINITE);
    }
}

//-----------------------------------------------------------------------------
// Wait for pending GPU work to complete.
// not interacting with swap chain.
//-----------------------------------------------------------------------------
void Scene::WaitForGpu()
{
    // Add a signal command to the queue.
    ThrowIfFailed(m_commandQueue->Signal(m_renderFence.Get(), m_renderFenceValue));

    // Instruct the fence to set the event object when the signal command completes.
    ThrowIfFailed(m_renderFence->SetEventOnCompletion(m_renderFenceValue, m_renderFenceEvent));
    m_renderFenceValue++;

    // Wait until the signal command has been processed.
    WaitForSingleObject(m_renderFenceEvent, INFINITE);
}

//-----------------------------------------------------------------------------
// initialize TileUpdateManager
//-----------------------------------------------------------------------------
void Scene::StartStreamingLibrary()
{
    TileUpdateManager::TileUpdateManagerDesc tumDesc;
    tumDesc.m_maxNumCopyBatches = m_args.m_numStreamingBatches;
    tumDesc.m_maxTileCopiesPerBatch = m_args.m_streamingBatchSize;
    tumDesc.m_maxTileCopiesInFlight = m_args.m_maxTilesInFlight;
    tumDesc.m_maxTileMappingUpdatesPerApiCall = m_args.m_maxTileUpdatesPerApiCall;
    tumDesc.m_swapChainBufferCount = SharedConstants::SWAP_CHAIN_BUFFER_COUNT;
    tumDesc.m_useDirectStorage = m_args.m_useDirectStorage;

    m_pTileUpdateManager = std::make_unique<TileUpdateManager>(m_device.Get(), m_commandQueue.Get(), tumDesc);

    for (UINT i = 0; i < m_args.m_numHeaps; i++)
    {
        m_sharedHeaps.push_back(m_pTileUpdateManager->CreateStreamingHeap(m_args.m_streamingHeapSize));
    }
}

//-----------------------------------------------------------------------------
// generate a random scale, position, and rotation
// also space the spheres so they do not touch
//-----------------------------------------------------------------------------
XMMATRIX Scene::SetSphereMatrix()
{
    static std::default_random_engine gen(42);
    static std::uniform_real_distribution<float> dis(-1, 1);

    const float MIN_SPHERE_SIZE = 1.f;
    const float MAX_SPHERE_SIZE = float(SharedConstants::MAX_SPHERE_SCALE);
    const float SPHERE_SPACING = float(100 + SharedConstants::MAX_SPHERE_SCALE) / 100.f;
    static std::uniform_real_distribution<float> scaleDis(MIN_SPHERE_SIZE, MAX_SPHERE_SIZE);

    bool tryAgain = true;
    UINT maxTries = 1000;

    XMMATRIX matrix = XMMatrixIdentity();

    while (tryAgain)
    {
        if (maxTries)
        {
            maxTries--;
        }
        else
        {
            MessageBox(0, L"Failed to fit planet in universe. Universe too small?", L"ERROR", MB_OK);
            exit(-1);
        }

        float sphereScale = scaleDis(gen) * SharedConstants::SPHERE_SCALE;

        float worldScale = SharedConstants::UNIVERSE_SIZE;

        float x = worldScale * std::abs(dis(gen));

        // position sphere far from terrain
        x += (MAX_SPHERE_SIZE + 2) * SharedConstants::SPHERE_SCALE;

        float rx = (2*XM_PI) * dis(gen);
        float ry = (2*XM_PI) * dis(gen);
        float rz = (2*XM_PI) * dis(gen);

        XMMATRIX xlate = XMMatrixTranslation(x, 0, 0);
        XMMATRIX rtate = XMMatrixRotationRollPitchYaw(rx, ry, rz);
        XMMATRIX scale = XMMatrixScaling(sphereScale, sphereScale, sphereScale);

        matrix = scale * xlate * rtate;

        tryAgain = false;

        // spread the spheres out
        {
            XMVECTOR p0 = matrix.r[3];
            float s0 = sphereScale;
            for (const auto o : m_objects)
            {
                if (o == m_pSky)
                {
                    continue;
                }

                XMVECTOR p1 = o->GetModelMatrix().r[3];
                float dist = XMVectorGetX(XMVector3LengthEst(p1 - p0));
                float s1 = XMVectorGetX(XMVector3LengthEst(o->GetModelMatrix().r[0]));

                // bigger planets are further apart
                if (dist < SPHERE_SPACING * (s0 + s1))
                {
                    tryAgain = true;
                    break;
                }
            }
        }
    }

    // pre-rotate to randomize axes
    float rx = (1.5f * XM_PI) * dis(gen);
    float ry = (2.5f * XM_PI) * dis(gen);
    float rz = (2.0f * XM_PI) * dis(gen); // rotation around polar axis of sphere model
    XMMATRIX rtate = XMMatrixRotationRollPitchYaw(rx, ry, rz);
    matrix = rtate * matrix;

    return matrix;
}

//-----------------------------------------------------------------------------
// progressively over multiple frames, if there are many
//-----------------------------------------------------------------------------
void Scene::LoadSpheres()
{
    // can't add or remove spheres before flushing all outstanding commands
    if (m_numSpheresLoaded != (UINT)m_args.m_numSpheres)
    {
        m_pTileUpdateManager->Finish();
    }

    if (m_numSpheresLoaded < (UINT)m_args.m_numSpheres)
    {
        // sphere descriptors start after the terrain descriptor
        CD3DX12_CPU_DESCRIPTOR_HANDLE descCPU = CD3DX12_CPU_DESCRIPTOR_HANDLE(m_srvHeap->GetCPUDescriptorHandleForHeapStart(), (UINT)DescriptorHeapOffsets::NumEntries, m_srvUavCbvDescriptorSize);

        // offset by all the spheres that have been loaded so far
        descCPU.Offset(m_numSpheresLoaded * (UINT)SceneObjects::Descriptors::NumEntries, m_srvUavCbvDescriptorSize);

        const UINT numSpheresToLoad = m_args.m_numSpheres - m_numSpheresLoaded;

        for (UINT i = 0; i < numSpheresToLoad; i++)
        {
            // this object's index-to-be
            UINT objectIndex = (UINT)m_objects.size();

            // put this resource into one of our shared heaps
            UINT heapIndex = objectIndex % m_sharedHeaps.size();
            auto pHeap = m_sharedHeaps[heapIndex];

            // grab the next texture
            UINT fileIndex = objectIndex % m_args.m_textures.size();
            const auto& textureFilename = m_args.m_textures[fileIndex];

            SceneObjects::BaseObject* o = nullptr;

            SphereGen::Properties sphereProperties;
            sphereProperties.m_numLat = m_args.m_sphereLat;
            sphereProperties.m_numLong = m_args.m_sphereLong;
            sphereProperties.m_mirrorU = true;

            // 3 options: sphere, earth, sky

            // sky
            if ((m_args.m_skyTexture.size()) && (nullptr == m_pSky)) // only 1 sky
            {
                m_pSky = new SceneObjects::Sky(
                    m_args.m_skyTexture, m_pTileUpdateManager.get(),
                    m_sharedHeaps[0], m_device.Get(), m_args.m_sampleCount, descCPU);
                o = m_pSky;
            }

            else if (nullptr == m_pTerrainSceneObject)
            {
                m_pTerrainSceneObject = new SceneObjects::Terrain(m_args.m_textureFilename, m_pTileUpdateManager.get(), pHeap, m_device.Get(), m_args.m_sampleCount, descCPU, m_args);
                m_terrainObjectIndex = objectIndex;
                o = m_pTerrainSceneObject;
            }

            // earth
            else if (m_args.m_earthTexture.size() && (m_args.m_earthTexture.size() < textureFilename.size()) &&
                (0 == textureFilename.compare(textureFilename.length() - m_args.m_earthTexture.length(), m_args.m_earthTexture.length(), m_args.m_earthTexture)))
            {
                if (nullptr == m_pEarth)
                {
                    sphereProperties.m_mirrorU = false;
                    o = new SceneObjects::Planet(textureFilename, m_pTileUpdateManager.get(), pHeap, m_device.Get(), m_args.m_sampleCount, descCPU, sphereProperties);
                    m_pEarth = o;
                }
                else
                {
                    o = new SceneObjects::Planet(textureFilename, m_pTileUpdateManager.get(), pHeap, m_device.Get(), descCPU, m_pEarth);
                }
                o->GetModelMatrix() = SetSphereMatrix();
            }

            // planet
            else
            {
                if (nullptr == m_pFirstSphere)
                {
                    sphereProperties.m_mirrorU = true;
                    o = new SceneObjects::Planet(textureFilename, m_pTileUpdateManager.get(), pHeap, m_device.Get(), m_args.m_sampleCount, descCPU, sphereProperties);
                    m_pFirstSphere = o;
                }
                else
                {
                    o = new SceneObjects::Planet(textureFilename, m_pTileUpdateManager.get(), pHeap, m_device.Get(), descCPU, m_pFirstSphere);
                }
                o->GetModelMatrix() = SetSphereMatrix();
            }
            m_objects.push_back(o);
            m_numSpheresLoaded++;

            // offset to the next sphere
            descCPU.Offset((UINT)SceneObjects::Descriptors::NumEntries, m_srvUavCbvDescriptorSize);
        }
    }
    // evict spheres?
    else if (m_numSpheresLoaded > (UINT)m_args.m_numSpheres)
    {
        WaitForGpu();
        while (m_numSpheresLoaded > (UINT)m_args.m_numSpheres)
        {
            auto i = m_objects.end();
            i--;

            SceneObjects::BaseObject* pObject = *i;

            delete pObject;
            m_objects.erase(i);

            if (m_pTerrainSceneObject == pObject)
            {
                DeleteTerrainViewers();
                m_pTerrainSceneObject = nullptr;
            }

            if (m_pFirstSphere == pObject)
            {
                m_pFirstSphere = nullptr;
            }

            if (m_pEarth == pObject)
            {
                m_pEarth = nullptr;
            }

            if (m_pSky == pObject)
            {
                m_pSky = nullptr;
            }

            m_numSpheresLoaded--;
        }
    }
}

//-----------------------------------------------------------------------------
// create MSAA color and depth targets
//-----------------------------------------------------------------------------
void Scene::CreateRenderTargets()
{
    D3D12_DEPTH_STENCIL_VIEW_DESC dsv = {};

    D3D12_RESOURCE_DESC desc = CD3DX12_RESOURCE_DESC::Tex2D(
        SharedConstants::SWAP_CHAIN_FORMAT,
        (UINT64)m_viewport.Width, (UINT)m_viewport.Height,
        1, 1, m_args.m_sampleCount);

    // create color buffer
    {
        D3D12_RESOURCE_DESC colorDesc = desc;
        colorDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;

        D3D12_CLEAR_VALUE clearValue = {};
        memcpy(&clearValue.Color, &m_clearColor, sizeof(m_clearColor));
        clearValue.Format = desc.Format;

        const auto heapProperties = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);
        ThrowIfFailed(m_device->CreateCommittedResource(
            &heapProperties,
            D3D12_HEAP_FLAG_NONE,
            &colorDesc,
            D3D12_RESOURCE_STATE_RENDER_TARGET,
            &clearValue,
            IID_PPV_ARGS(&m_colorBuffer)));
    }

    // create depth buffer
    {
        D3D12_RESOURCE_DESC depthDesc = desc;
        depthDesc.Format = SharedConstants::DEPTH_FORMAT;
        depthDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;

        D3D12_CLEAR_VALUE clearValue = {};
        clearValue.DepthStencil.Depth = 1.0f;
        clearValue.Format = SharedConstants::DEPTH_FORMAT;

        const auto heapProperties = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);
        ThrowIfFailed(m_device->CreateCommittedResource(
            &heapProperties,
            D3D12_HEAP_FLAG_NONE,
            &depthDesc,
            D3D12_RESOURCE_STATE_DEPTH_WRITE,
            &clearValue,
            IID_PPV_ARGS(&m_depthBuffer)));

        dsv.Format = depthDesc.Format;
    }

    if (1 == m_args.m_sampleCount)
    {
        dsv.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
    }
    else
    {
        dsv.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2DMS;
    }

    CD3DX12_CPU_DESCRIPTOR_HANDLE rtvDescriptor(m_rtvHeap->GetCPUDescriptorHandleForHeapStart(), m_swapBufferCount, m_rtvDescriptorSize);
    CD3DX12_CPU_DESCRIPTOR_HANDLE dsvDescriptor(m_dsvHeap->GetCPUDescriptorHandleForHeapStart());

    m_device->CreateRenderTargetView(m_colorBuffer.Get(), nullptr, rtvDescriptor);
    m_device->CreateDepthStencilView(m_depthBuffer.Get(), &dsv, dsvDescriptor);
}

//-----------------------------------------------------------------------------
// 1 static and 1 dynamic constant buffers
// NOTE: do this within LoadAssets, as it will create a staging resource and rely on command list submission
//-----------------------------------------------------------------------------
void Scene::CreateConstantBuffers()
{
    // dynamic constant buffer
    {
        UINT bufferSize = sizeof(FrameConstantData);
        const UINT multipleSize = 256; // required
        bufferSize = (bufferSize + multipleSize - 1) / multipleSize;
        bufferSize *= multipleSize;

        const auto heapProperties = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD);
        const auto resourceDesc = CD3DX12_RESOURCE_DESC::Buffer(bufferSize);
        ThrowIfFailed(m_device->CreateCommittedResource(
            &heapProperties,
            D3D12_HEAP_FLAG_NONE,
            &resourceDesc,
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&m_frameConstantBuffer)));

        CD3DX12_RANGE readRange(0, bufferSize);
        ThrowIfFailed(m_frameConstantBuffer->Map(0, &readRange, reinterpret_cast<void**>(&m_pFrameConstantData)));

        m_pFrameConstantData->g_lightDir = XMFLOAT4(-0.449135751f, 0.656364977f, 0.25f, 0);
        m_pFrameConstantData->g_lightColor = XMFLOAT4(1, 1, 1, 40.0f);
        m_pFrameConstantData->g_specColor = XMFLOAT4(1, 1, 1, 1);

        D3D12_CONSTANT_BUFFER_VIEW_DESC constantBufferView = {};
        constantBufferView.SizeInBytes = bufferSize;
        constantBufferView.BufferLocation = m_frameConstantBuffer->GetGPUVirtualAddress();
        m_device->CreateConstantBufferView(&constantBufferView, CD3DX12_CPU_DESCRIPTOR_HANDLE(
            m_srvHeap->GetCPUDescriptorHandleForHeapStart(), (UINT)DescriptorHeapOffsets::FRAME_CBV, m_srvUavCbvDescriptorSize));
    }
}

//-----------------------------------------------------------------------------
    // the sampler, which can be adjusted by the UI
//-----------------------------------------------------------------------------
void Scene::CreateSampler()
{
    D3D12_DESCRIPTOR_HEAP_DESC desc = {};
    desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_SAMPLER;
    desc.NumDescriptors = 1; // only need the one for the single feedback map
    desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    ThrowIfFailed(m_device->CreateDescriptorHeap(&desc, IID_PPV_ARGS(&m_samplerHeap)));
    m_samplerHeap->SetName(L"m_samplerHeap");
}

//-----------------------------------------------------------------------------
// sampler used for accessing feedback map
// can change dynamically with UI slider
//-----------------------------------------------------------------------------
void Scene::SetSampler()
{
    D3D12_SAMPLER_DESC samplerDesc{};
    samplerDesc.AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    samplerDesc.AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    samplerDesc.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;

    if (m_args.m_anisotropy < 2)
    {
        samplerDesc.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    }
    else
    {
        samplerDesc.MaxAnisotropy = std::min((UINT)D3D12_MAX_MAXANISOTROPY, m_args.m_anisotropy);
        samplerDesc.Filter = D3D12_FILTER_ANISOTROPIC;
    }

    samplerDesc.MaxLOD = FLT_MAX;
    samplerDesc.MipLODBias = m_args.m_lodBias;

    CD3DX12_CPU_DESCRIPTOR_HANDLE descHandle(m_samplerHeap->GetCPUDescriptorHandleForHeapStart());
    m_device->CreateSampler(&samplerDesc, descHandle);
}

//----------------------------------------------------------
// should we drain tiles?
//----------------------------------------------------------
void Scene::DrainTiles()
{
    bool drainTiles = false;

    static int previousVisualizationMode = m_args.m_dataVisualizationMode;
    if (previousVisualizationMode != m_args.m_dataVisualizationMode)
    {
        m_pTileUpdateManager->SetVisualizationMode((UINT)m_args.m_dataVisualizationMode);
        previousVisualizationMode = m_args.m_dataVisualizationMode;
        drainTiles = true;
    }

    if (m_args.m_drainTiles)
    {
        m_args.m_drainTiles = false;
        drainTiles = true;
    }

    if (drainTiles)
    {
        m_pTileUpdateManager->Finish();
        for (auto m : m_objects)
        {
            m->GetStreamingResource()->ClearAllocations();
        }
    }
}

//----------------------------------------------------------
// time-limit the number of feedback resolves on the GPU
// by keeping a running average of the time to resolve feedback
// and only calling QueueFeedback() for a subset of the StreamingResources
//----------------------------------------------------------
UINT Scene::DetermineMaxNumFeedbackResolves()
{
    UINT maxNumFeedbackResolves = 0;

    if (m_args.m_updateEveryObjectEveryFrame)
    {
        maxNumFeedbackResolves = (UINT)m_objects.size();
    }
    else if (m_args.m_maxTileUpdatesPerFrame)
    {
        maxNumFeedbackResolves = 10;

        // how long did it take
        const float feedbackTime = 1000.f * m_pTileUpdateManager->GetGpuTime();

        if (feedbackTime > 0)
        {
            // keep a running average of feedback time
            static AverageOver feedbackTimes;

            float avgTimePerObject = feedbackTime / std::max(UINT(1), m_prevNumFeedbackObjects[m_frameIndex]);
            feedbackTimes.Update(avgTimePerObject);

            // # objects to meet target time
            maxNumFeedbackResolves = std::max(UINT(1), UINT(m_args.m_maxGpuFeedbackTimeMs / feedbackTimes.Get()));
        }
    }

    return maxNumFeedbackResolves;
}

//-----------------------------------------------------------------------------
// draw all objects
// uses the min-mip-map created using Sampler Feedback on the GPU
// to recommend updates to the internal memory map managed by the CPU
// returns an actual occupancy min-mip-map
// 
// feedback may only written for a subset of resources depending on GPU feedback timeout
//-----------------------------------------------------------------------------
void Scene::DrawObjects()
{
    GpuScopeTimer gpuScopeTimer(m_pGpuTimer, m_commandList.Get(), "GPU Frame Time");
    if (0 == m_objects.size())
    {
        return;
    }

    SceneObjects::DrawParams drawParams;
    drawParams.m_sharedMinMipMap = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_srvHeap->GetGPUDescriptorHandleForHeapStart(), (UINT)DescriptorHeapOffsets::SHARED_MIN_MIP_MAP, m_srvUavCbvDescriptorSize);
    drawParams.m_constantBuffers = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_srvHeap->GetGPUDescriptorHandleForHeapStart(), (UINT)DescriptorHeapOffsets::FRAME_CBV, m_srvUavCbvDescriptorSize);
    drawParams.m_samplers = m_samplerHeap->GetGPUDescriptorHandleForHeapStart();
    drawParams.m_projection = m_projection;
    drawParams.m_view = m_viewMatrix;
    drawParams.m_viewInverse = m_viewMatrixInverse;

    const D3D12_GPU_DESCRIPTOR_HANDLE srvBaseGPU = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_srvHeap->GetGPUDescriptorHandleForHeapStart(), (UINT)DescriptorHeapOffsets::NumEntries, m_srvUavCbvDescriptorSize);

    //------------------------------------------------------------------------------------
    // set feedback state on each object
    // objects with feedback enabled will queue feedback resolve on the TileUpdateManager
    // objects without feedback enabled will not call WriteSamplerFeedback()
    //------------------------------------------------------------------------------------
    {
        UINT maxNumFeedbackResolves = DetermineMaxNumFeedbackResolves();

        // clamp in case # objects changed
        m_queueFeedbackIndex = m_queueFeedbackIndex % m_objects.size();

        // loop over n objects starting with the range that we want to get sampler feedback from, then wrap around.
        UINT numObjects = m_queueFeedbackIndex + (UINT)m_objects.size();
        UINT numFeedbackObjects = 0;
        for (UINT i = m_queueFeedbackIndex; i < numObjects; i++)
        {
            auto o = m_objects[i % (UINT)m_objects.size()];

            // get sampler feedback for this object?
            bool queueFeedback = false;
            if (numFeedbackObjects < maxNumFeedbackResolves)
            {
                queueFeedback = true;
                numFeedbackObjects++;
            }
            o->SetFeedbackEnabled(queueFeedback);
        }
        // start feedback where we left off this time.
        // note m_queueFeedbackIndex will be adjusted to # of objects next time, above
        m_queueFeedbackIndex += numFeedbackObjects;
        // remember how many resolves were queued for the running average
        m_prevNumFeedbackObjects[m_frameIndex] = numFeedbackObjects;
    }

    // draw the objects in the same order each time
    auto descriptorBase = CD3DX12_GPU_DESCRIPTOR_HANDLE(srvBaseGPU);
    for (auto o : m_objects)
    {
        drawParams.m_srvBaseGPU = descriptorBase;
        descriptorBase.Offset((UINT)SceneObjects::Descriptors::NumEntries, m_srvUavCbvDescriptorSize);

        o->Draw(m_commandList.Get(), drawParams);
    }
}

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
void Scene::MsaaResolve()
{
    ID3D12Resource* pRenderTarget = m_renderTargets[m_frameIndex].Get();

    D3D12_RESOURCE_BARRIER barriers[] = {
        CD3DX12_RESOURCE_BARRIER::Transition(m_colorBuffer.Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_RESOLVE_SOURCE),
        CD3DX12_RESOURCE_BARRIER::Transition(pRenderTarget, D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RESOLVE_DEST)
    };

    m_commandList->ResourceBarrier(_countof(barriers), barriers);

    m_commandList->ResolveSubresource(pRenderTarget, 0, m_colorBuffer.Get(), 0, pRenderTarget->GetDesc().Format);

    std::swap(barriers[0].Transition.StateBefore, barriers[0].Transition.StateAfter);
    barriers[1].Transition.StateBefore = barriers[1].Transition.StateAfter;
    barriers[1].Transition.StateAfter = D3D12_RESOURCE_STATE_RENDER_TARGET;

    m_commandList->ResourceBarrier(_countof(barriers), barriers);

    // after resolve, set the swap chain as the render target
    D3D12_CPU_DESCRIPTOR_HANDLE rtvHandle = CD3DX12_CPU_DESCRIPTOR_HANDLE(m_rtvHeap->GetCPUDescriptorHandleForHeapStart(), m_frameIndex, m_rtvDescriptorSize);
    m_commandList->OMSetRenderTargets(1, &rtvHandle, FALSE, nullptr);
}

//-------------------------------------------------------------------------
// capture an image of the render target
//-------------------------------------------------------------------------
void Scene::ScreenShot(std::wstring& in_fileName) const
{
    std::wstring filename(in_fileName);
    filename += L".png";
    WindowCapture::CaptureRenderTarget(m_renderTargets[m_frameIndex].Get(), m_commandQueue.Get(), filename);
}

//-------------------------------------------------------------------------
// write internal batch timings
//-------------------------------------------------------------------------
void Scene::WriteBatchTimes(std::wstring& in_fileName) const
{
    WriteCSV csvFile(in_fileName);

    csvFile << GetCommandLineW();

    csvFile << "\n\nStreaming (ms)\n"
        << "-----------------------------------------------------------------------------------------------------------\n"
        << "cpu_copy gpu_copy total_copy_time mapping total_time tiles_copied tiles_unmapped\n"
        << "-----------------------------------------------------------------------------------------------------------\n";

    auto& streamingTimes = m_pTileUpdateManager->GetBatchTimes();
    for (const auto& b : streamingTimes)
    {
        csvFile
            << b.m_cpuTime * 1000.f << " "
            << b.m_gpuTime * 1000.f << " "
            << b.m_copyTime * 1000.f << " "
            << b.m_mappingTime * 1000.f << " "
            << b.m_totalTime * 1000.f << " "
            << b.m_numTilesCopied << " "
            << b.m_numTilesUnMapped
            << std::endl;
    }
}

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
void Scene::GatherStatistics(float in_cpuProcessFeedbackTime, float in_gpuProcessFeedbackTime)
{
    // NOTE: streaming isn't aware of frame time.
    // these numbers are approximately a measure of the number of operations during the last frame
    UINT numEvictions = m_pTileUpdateManager->GetTotalNumEvictions();
    UINT numUploads = m_pTileUpdateManager->GetTotalNumUploads();

    m_numEvictionsPreviousFrame = numEvictions - m_numTotalEvictions;
    m_numUploadsPreviousFrame = numUploads - m_numTotalUploads;

    m_numTotalEvictions = numEvictions;
    m_numTotalUploads = numUploads;

    // statistics gathering
    if (m_args.m_timingFrameFileName.size() &&
        (m_frameNumber > m_args.m_timingStartFrame) &&
        (m_frameNumber <= m_args.m_timingStopFrame))
    {
        m_csvFile->Append(m_renderThreadTimes, m_updateFeedbackTimes,
            m_numUploadsPreviousFrame, m_numEvictionsPreviousFrame,
            m_prevNumFeedbackObjects[m_frameIndex],
            // Note: these may be off by 1 frame, but probably good enough
            in_cpuProcessFeedbackTime, in_gpuProcessFeedbackTime);

        if (m_frameNumber == m_args.m_timingStopFrame)
        {
            DebugPrint(L"Gathering final statistics before exitting\n");

            m_csvFile->WriteEvents(m_hwnd);
            m_csvFile->close();
            m_csvFile = nullptr;
        }
    }

    // always exit if the stop frame is set
    if ((m_args.m_timingStopFrame > 0) && (m_frameNumber >= m_args.m_timingStopFrame))
    {
        PostQuitMessage(0);
    }
}

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
void Scene::Animate()
{
    if (m_args.m_waitForAssetLoad)
    {
        for (const auto& o : m_objects)
        {
            if (!o->GetPackedMipsPresent())
            {
                return; // do not animate or, for statistics purposes, increment frame #
            }
        }
    }

    // for statistics gathering
    m_frameNumber++;

    // animate camera
    if (m_args.m_cameraAnimationRate)
    {
        m_args.m_cameraUpLock = false;

        if (m_args.m_cameraPaintMixer)
        {
            m_args.m_cameraRollerCoaster = (0x08 & m_frameNumber);
        }

        static float theta = -XM_PIDIV2;
        const float delta = 0.01f * m_args.m_cameraAnimationRate;
        float radius = 5.5 * (float)SharedConstants::CAMERA_ANIMATION_RADIUS;

        if (m_args.m_cameraRollerCoaster)
        {
            radius /= 2.f;
        }

        theta += delta;

        float x = radius * std::cos(theta);
        float y = 2 * radius * std::cos(theta / 4);
        float z = radius * std::sin(theta);

        if (m_args.m_cameraRollerCoaster)
        {
            static XMVECTOR previous = XMVectorSet(0, 0, 0, 0);
            XMVECTOR pos = XMVectorSet(x * std::sin(theta / 4), y / 2, z / 3, 1);
            XMVECTOR lookTo = XMVector3Normalize(pos - previous);
            lookTo = XMVectorSetW(lookTo, 1.0f);

            XMVECTOR vUpVec = XMVectorSet(0.0f, 1.0f, 0.0f, 1.0f);

            m_viewMatrix = XMMatrixLookToLH(pos, lookTo, vUpVec);

            previous = pos;
        }
        else
        {
            XMVECTOR pos = XMVectorSet(x, y, z, 1);
            m_viewMatrix = XMMatrixLookAtLH(pos, XMVectorSet(0, 0, 0, 0), XMVectorSet(0.0f, 1.0f, 0.0f, 1.0f));
        }

        XMVECTOR pDet;
        m_viewMatrixInverse = XMMatrixInverse(&pDet, m_viewMatrix);
    }

    // spin objects
    if (m_args.m_animationRate)
    {
        // WARNING: if rotation amount is based on frametime, we get a feedback situation:
        // longer frametime leads to larger rotation per frame
        // a large change to frame contents causes many evictions/loads, affecting frametime...
        // an increasing frametime increases rotation/frame which increases frame time!

        //float rotation = m_args.m_animationRate * GetFrameTime() / 200.0f;
        float rotation = m_args.m_animationRate * 0.01f;

        for (auto o : m_objects)
        {
            if (m_pTerrainSceneObject == o)
            {
                o->GetModelMatrix() = DirectX::XMMatrixRotationY(rotation) * o->GetModelMatrix();
            }
            else if (m_pSky != o)
            {
                o->GetModelMatrix() = DirectX::XMMatrixRotationZ(rotation) * o->GetModelMatrix();
            }
        }
    }
}

//-------------------------------------------------------------------------
// create various windows to inspect terrain object resources
//-------------------------------------------------------------------------
void Scene::CreateTerrainViewers()
{
    ASSERT(m_pTerrainSceneObject);
    if (nullptr == m_pTextureViewer)
    {
        UINT heapOffset = (UINT)DescriptorHeapOffsets::NumEntries +
            (m_terrainObjectIndex * (UINT)SceneObjects::Descriptors::NumEntries) +
            (UINT)SceneObjects::Descriptors::HeapOffsetTexture;

        // create viewer for the streaming resource
        m_pTextureViewer = new TextureViewer(
            m_pTerrainSceneObject->GetTiledResource(),
            SharedConstants::SWAP_CHAIN_FORMAT, m_srvHeap.Get(),
            heapOffset);
    }
#if RESOLVE_TO_TEXTURE
    // create viewer for the resolved feedback
    if (nullptr == m_pFeedbackViewer)
    {
        UINT feedbackWidth = m_pTerrainSceneObject->GetStreamingResource()->GetMinMipMapWidth();
        UINT feedbackHeight = m_pTerrainSceneObject->GetStreamingResource()->GetMinMipMapHeight();

        m_pFeedbackViewer = new BufferViewer(
            m_pTerrainSceneObject->GetResolvedFeedback(),
            feedbackWidth, feedbackHeight, feedbackWidth, 0,
            SharedConstants::SWAP_CHAIN_FORMAT);
    }
#endif

    // NOTE: shared minmipmap will be nullptr until after TUM::BeginFrame()
    // NOTE: the data will be delayed by 1 + 1 frame for each swap buffer (e.g. 3 for double-buffering)
    if (nullptr == m_pMinMipMapViewer)
    {
        // create min-mip-map viewer
        UINT feedbackWidth = m_pTerrainSceneObject->GetStreamingResource()->GetMinMipMapWidth();
        UINT feedbackHeight = m_pTerrainSceneObject->GetStreamingResource()->GetMinMipMapHeight();

        // FIXME: bufferview can't be created until after beginframe!
        m_pMinMipMapViewer = new BufferViewer(
            m_pTerrainSceneObject->GetMinMipMap(),
            feedbackWidth, feedbackHeight, feedbackWidth,
            m_pTerrainSceneObject->GetStreamingResource()->GetMinMipMapOffset(),
            SharedConstants::SWAP_CHAIN_FORMAT,
            m_srvHeap.Get(), (INT)DescriptorHeapOffsets::SHARED_MIN_MIP_MAP);
    }
}

//-------------------------------------------------------------------------
// delete various windows to inspect terrain object resources
//-------------------------------------------------------------------------
void Scene::DeleteTerrainViewers()
{
    if (m_pTextureViewer)
    {
        delete m_pTextureViewer;
        m_pTextureViewer = nullptr;
    }
    if (m_pFeedbackViewer)
    {
        delete m_pFeedbackViewer;
        m_pFeedbackViewer = nullptr;
    }
    if (m_pMinMipMapViewer)
    {
        delete m_pMinMipMapViewer;
        m_pMinMipMapViewer = nullptr;
    }
}

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
bool Scene::Draw()
{
    if (m_deviceRemoved)
    {
        return false;
    }

    if (m_useDirectStorage != m_args.m_useDirectStorage)
    {
        m_useDirectStorage = m_args.m_useDirectStorage;
        m_pTileUpdateManager->UseDirectStorage(m_useDirectStorage);
    }

    DrainTiles();

    // load more spheres?
    // SceneResource destruction/creation must be done outside of BeginFrame/EndFrame
    LoadSpheres();

    m_renderThreadTimes.Set(RenderEvents::FrameBegin);

    // prepare to update Feedback & stream textures
    D3D12_CPU_DESCRIPTOR_HANDLE minmipmapDescriptor = CD3DX12_CPU_DESCRIPTOR_HANDLE(m_srvHeap->GetCPUDescriptorHandleForHeapStart(), (UINT)DescriptorHeapOffsets::SHARED_MIN_MIP_MAP, m_srvUavCbvDescriptorSize);
    m_pTileUpdateManager->BeginFrame(m_srvHeap.Get(), minmipmapDescriptor);

    Animate();

    //-------------------------------------------
    // frustum visualization
    //-------------------------------------------
    if (m_showFrustum != m_args.m_visualizeFrustum)
    {
        m_showFrustum = m_args.m_visualizeFrustum;
        static UINT maxUpdates = m_args.m_maxTileUpdatesPerFrame;
        static float samplerLodBias = m_args.m_lodBias;

        // stop updating while the frustum is shown
        if (m_showFrustum)
        {
            // stop spinning
            m_args.m_animationRate = 0;

            XMVECTOR lookDir = m_viewMatrixInverse.r[2];
            XMVECTOR pos = m_viewMatrixInverse.r[3];
                
            // scale to something within universe scale
            float scale = SharedConstants::SPHERE_SCALE * 2.5;

            m_pFrustumViewer->SetView(m_viewMatrixInverse, scale);

            maxUpdates = m_args.m_maxTileUpdatesPerFrame;
            m_args.m_maxTileUpdatesPerFrame = 0;
            m_args.m_lodBias = -5.0f;
        }
        else
        {
            m_args.m_maxTileUpdatesPerFrame = maxUpdates;
            m_args.m_lodBias = samplerLodBias;
        }
    }

    //-------------------------------------------
    // set rendering state
    //-------------------------------------------
    {
        m_commandAllocators[m_frameIndex]->Reset();
        m_commandList->Reset((ID3D12CommandAllocator*)m_commandAllocators[m_frameIndex].Get(), nullptr);

        // the first 0..(m_swapBufferCount-1) rtv handles point to the swap chain
        // there is one rtv in the rtv heap that points to the color buffer, at offset m_swapBufferCount:
        const CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(m_rtvHeap->GetCPUDescriptorHandleForHeapStart(), m_swapBufferCount, m_rtvDescriptorSize);
        const CD3DX12_CPU_DESCRIPTOR_HANDLE dsvHandle(m_dsvHeap->GetCPUDescriptorHandleForHeapStart());
        ID3D12DescriptorHeap* ppHeaps[] = { m_srvHeap.Get(), m_samplerHeap.Get() };
        m_commandList->SetDescriptorHeaps(_countof(ppHeaps), ppHeaps);

        m_commandList->OMSetRenderTargets(1, &rtvHandle, TRUE, &dsvHandle);

        m_commandList->ClearRenderTargetView(rtvHandle, m_clearColor, 0, nullptr);
        m_commandList->ClearDepthStencilView(dsvHandle, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);

        m_commandList->RSSetViewports(1, &m_viewport);
        m_commandList->RSSetScissorRects(1, &m_scissorRect);

        SetSampler();
        m_pFrameConstantData->g_view = m_viewMatrix;
        m_pFrameConstantData->g_visualizeFeedback = m_args.m_visualizeMinMip;

        if (m_args.m_lightFromView)
        {
            auto transposeView = DirectX::XMMatrixTranspose(m_viewMatrix);
            DirectX::XMVECTOR lookDir = DirectX::XMVectorNegate(transposeView.r[2]);
            m_pFrameConstantData->g_lightDir = (XMFLOAT4&)lookDir;
        }
        else
        {
            m_pFrameConstantData->g_lightDir = XMFLOAT4(-0.449135751f, 0.656364977f, 0.25f, 0);
        }
    }

    DrawObjects();

    if (m_showFrustum)
    {
        XMMATRIX combinedTransform = XMMatrixMultiply(m_viewMatrix, m_projection);
        m_pFrustumViewer->Draw(m_commandList.Get(), combinedTransform, m_fieldOfView, m_aspectRatio);
    }

    //-------------------------------------------
    // MSAA resolve
    //-------------------------------------------
    {
        GpuScopeTimer msaaScopeTimer(m_pGpuTimer, m_commandList.Get(), "GPU MSAA resolve");
        MsaaResolve();
    }

    //-------------------------------------------
    // Display various textures
    //-------------------------------------------
    if (m_args.m_showFeedbackMaps && (nullptr != m_pTerrainSceneObject))
    {
        CreateTerrainViewers();

        float windowHeight = m_viewport.Height / 4.0f;
        UINT numWindows = UINT(m_viewport.Width) / (UINT)windowHeight;
        UINT numMips = std::max(UINT(1), numWindows);

        float dim = m_viewport.Width / float(numMips);
        DirectX::XMFLOAT2 windowSize(dim, dim);

        if (m_args.m_showFeedbackMapVertical)
        {
            float guiHeight = 0;
            if (m_args.m_showUI)
            {
                guiHeight = m_pGui->GetHeight();
                UINT guiWidth = (UINT)m_pGui->GetWidth();
                numWindows = UINT(m_viewport.Height - guiHeight) / guiWidth;
                numMips = std::max((UINT)3, numWindows);
            }
            dim = float(m_viewport.Height - guiHeight) / numMips;
            windowSize = DirectX::XMFLOAT2(dim, dim);
        }
        DirectX::XMFLOAT2 windowPos(0, windowSize.y);

        m_pTextureViewer->Draw(m_commandList.Get(), windowPos, windowSize,
            m_viewport,
            m_args.m_visualizationBaseMip, numMips,
            m_args.m_showFeedbackMapVertical);

        // residency map
        windowPos = DirectX::XMFLOAT2(m_viewport.Width - windowSize.x, m_viewport.Height);

        if (m_args.m_showFeedbackViewer)
        {
            m_pMinMipMapViewer->Draw(m_commandList.Get(), windowPos, windowSize, m_viewport);

            // min mip feedback
#if RESOLVE_TO_TEXTURE
            windowPos.x -= (5 + windowSize.x);
            m_pFeedbackViewer->Draw(m_commandList.Get(), windowPos, windowSize, m_viewport);
#endif
        }
    }

    //-------------------------------------------
    // Display UI
    //-------------------------------------------
    float cpuProcessFeedbackTime = m_pTileUpdateManager->GetProcessFeedbackTime();
    float gpuProcessFeedbackTime = m_pTileUpdateManager->GetGpuTime();
    float gpuDrawTime = m_pGpuTimer->GetTimes()[0].first; // frame draw time
    if (m_args.m_showUI)
    {
        // note: TextureViewer and BufferViewer may have internal descriptor heaps
        ID3D12DescriptorHeap* ppHeaps[] = { m_srvHeap.Get(), m_samplerHeap.Get() };
        m_commandList->SetDescriptorHeaps(_countof(ppHeaps), ppHeaps);

        UINT numTilesVirtual = 0;
        for (auto& o : m_objects)
        {
            numTilesVirtual += o->GetStreamingResource()->GetNumTilesVirtual();
        }

        UINT numTilesCommitted = 0;
        for (auto h : m_sharedHeaps)
        {
            numTilesCommitted += h->GetAllocator().GetNumAllocated();
        }

        Gui::DrawParams guiDrawParams;
        guiDrawParams.m_gpuDrawTime = gpuDrawTime;
        guiDrawParams.m_gpuFeedbackTime = gpuProcessFeedbackTime;
        {
            // pass in raw cpu frame time and raw # uploads. GUI will keep a running average of bandwidth
            auto a = m_renderThreadTimes.GetLatest();
            guiDrawParams.m_cpuDrawTime = a.Get(RenderEvents::TumEndFrameBegin) - a.Get(RenderEvents::FrameBegin);
        }
        guiDrawParams.m_cpuFeedbackTime = cpuProcessFeedbackTime;
        if (m_pTerrainSceneObject)
        {
            guiDrawParams.m_scrollMipDim = m_pTerrainSceneObject->GetStreamingResource()->GetTiledResource()->GetDesc().MipLevels;
        }
        guiDrawParams.m_numTilesUploaded = m_numUploadsPreviousFrame;
        guiDrawParams.m_numTilesEvicted = m_numEvictionsPreviousFrame;
        guiDrawParams.m_numTilesCommitted = numTilesCommitted;
        guiDrawParams.m_numTilesVirtual = numTilesVirtual;
        guiDrawParams.m_totalHeapSize = m_args.m_streamingHeapSize * (UINT)m_sharedHeaps.size();
        guiDrawParams.m_windowHeight = m_args.m_windowHeight;

        if (m_args.m_uiModeMini)
        {
            m_pGui->DrawMini(m_commandList.Get(), guiDrawParams);
        }
        else
        {
            m_pGui->Draw(m_commandList.Get(), m_args, guiDrawParams);
        }
    }

    //-------------------------------------------
    // execute command lists
    //-------------------------------------------
    bool success = true;
    {
        D3D12_RESOURCE_BARRIER barrier = CD3DX12_RESOURCE_BARRIER::Transition(m_renderTargets[m_frameIndex].Get(),
            D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT);
        m_commandList->ResourceBarrier(1, &barrier);

        m_pGpuTimer->ResolveAllTimers(m_commandList.Get());
        m_commandList->Close();

        m_renderThreadTimes.Set(RenderEvents::TumEndFrameBegin);
        auto commandLists = m_pTileUpdateManager->EndFrame();
        m_renderThreadTimes.Set(RenderEvents::TumEndFrame);

        ID3D12CommandList* pCommandLists[] = { commandLists.m_beforeDrawCommands, m_commandList.Get(), commandLists.m_afterDrawCommands };
        m_commandQueue->ExecuteCommandLists(_countof(pCommandLists), pCommandLists);

        //-------------------------------------------
        // Present the frame.
        //-------------------------------------------
        UINT syncInterval = m_args.m_vsyncEnabled ? 1 : 0;
        UINT presentFlags = 0;
        if ((m_windowedSupportsTearing) && (!m_fullScreen) && (0 == syncInterval))
        {
            presentFlags = DXGI_PRESENT_ALLOW_TEARING;
        }
        success = IsDeviceOk(m_swapChain->Present(syncInterval, presentFlags));

        // gather statistics before moving to next frame
        GatherStatistics(cpuProcessFeedbackTime, gpuProcessFeedbackTime);

        m_renderThreadTimes.Set(RenderEvents::WaitOnFencesBegin);

        MoveToNextFrame();
        m_renderThreadTimes.Set(RenderEvents::FrameEnd);
    }

    m_renderThreadTimes.NextFrame();
    return success;
}