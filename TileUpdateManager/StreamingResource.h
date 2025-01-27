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


/*=============================================================================
Usage:

After creating a TileUpdateManager, a TextureStreamer, and a Streaming::Heap,
use TileUpdateManager::CreateStreamingResource()
=============================================================================*/

#pragma once

#include <vector>
#include <d3d12.h>
#include <string>

#include "InternalResources.h"

namespace Streaming
{
    class TileUpdateManagerBase;
    struct UpdateList;
    class XeTexture;
    class Heap;
    class FileHandle;

    //=============================================================================
    // unpacked mips are dynamically loaded/evicted, preserving a min-mip-map
    // packed mips are not evicted from the heap (as little as 1 tile for a 16k x 16k texture)
    //=============================================================================
    class StreamingResourceBase
    {
    public:
        // share heap & other resources with another TileUpdateManager
        StreamingResourceBase(
            // method that will fill a tile-worth of bits, for streaming
            const std::wstring& in_filename,
            Streaming::FileHandle* in_pFileHandle,
            // share heap and upload buffers with other InternalResources
            Streaming::TileUpdateManagerBase* in_pTileUpdateManager,
            Heap* in_pHeap);

        virtual ~StreamingResourceBase();

        // called whenever a new StreamingResource is created - even one other than "this"
        void SetResidencyMapOffsetBase(UINT in_residencyMapOffsetBase);

        // called when creating/changing FileStreamer
        void SetFileHandle(const class DataUploader* in_pDataUploader);

        //-------------------------------------
        // begin called by TUM::EndFrame()
        // note: that is, called once per frame
        //-------------------------------------

        // Called on every object every frame
        // exits fast if tile residency has not changed (due to addmap or decmap)
        void UpdateMinMipMap();

        // returns true if packed mips are loaded
        // NOTE: this query will only return true one time
        bool GetPackedMipsNeedTransition();

        // the following are called only if the application made a feedback request for the object:

        // called before draw to clear the feedback map
        void ClearFeedback(ID3D12GraphicsCommandList* in_pCmdList, const D3D12_GPU_DESCRIPTOR_HANDLE in_gpuDescriptor);

        ID3D12Resource* GetOpaqueFeedback() { return m_resources->GetOpaqueFeedback(); }

        // call after drawing to get feedback
        void ResolveFeedback(ID3D12GraphicsCommandList1* out_pCmdList);

#if RESOLVE_TO_TEXTURE
        // call after resolving to read back to CPU
        void ReadbackFeedback(ID3D12GraphicsCommandList* out_pCmdList);

        // TUM needs this for barrier before/after copy
        ID3D12Resource* GetResolvedFeedback() { return m_resources->GetResolvedFeedback(); }
#endif

        // TUM needs this for barrier on packed mips
        ID3D12Resource* GetTiledResource() const { return m_resources->GetTiledResource(); }

        //-------------------------------------
        // end called by TUM::EndFrame()
        //-------------------------------------

        //-------------------------------------
        // called by TUM::ProcessFeedbackThread
        //-------------------------------------

        // indicate the render frame has advanced
        // Useful in particular for preventing evictions of in-flight data
        void NextFrame() { m_pendingEvictions.NextFrame(); }

        // if a feedback buffer is ready, process it to generate lists of tiles to load/evict
        void ProcessFeedback(UINT64 in_frameFenceCompletedValue);

        // try to load/evict tiles. only queue evictions once per frame.
        void QueueTiles();

        bool IsStale()
        {
            return (m_pendingTileLoads.size() || m_pendingEvictions.GetReadyToEvict().size());
        }

        bool InitPackedMips();
        //-------------------------------------
        // end called by TUM::ProcessFeedbackThread
        //-------------------------------------

        UINT GetNumTilesWidth() const { return m_tileReferencesWidth; }
        UINT GetNumTilesHeight() const { return m_tileReferencesHeight; }

    protected:
        // object that streams data from a file
        std::unique_ptr<Streaming::XeTexture> m_pTextureStreamer;
        std::unique_ptr<Streaming::InternalResources> m_resources;
        std::unique_ptr<Streaming::FileHandle> m_pFileHandle;
        const std::wstring m_filename;
        Streaming::Heap* m_pHeap{ nullptr };

        void SetResidencyChanged();

        // packed mip status
        enum class PackedMipStatus : UINT32
        {
            UNINITIALIZED = 0, // have we requested packed mips yet?
            HEAP_RESERVED,     // heap spaced reserved
            REQUESTED,         // copy requested
            NEEDS_TRANSITION,  // copy complete, transition to readable
            RESIDENT           // mapped, loaded, and transitioned to pixel shader resource
        };
        PackedMipStatus m_packedMipStatus{ PackedMipStatus::UNINITIALIZED };

        //==================================================
        // TileMappingState keeps reference counts and heap indices for resources in a min-mip-map
        //==================================================
        class TileMappingState
        {
        public:
            void Init(UINT in_numMips, const D3D12_SUBRESOURCE_TILING* in_pTiling);

            UINT GetNumSubresources() const { return (UINT)m_refcounts.size(); }


            // 4 states are encoded by the residency state and ref count:
            // residency | refcount | tile state
            // ----------+----------+------------------
            //      0    |    0     | not resident (data not resident & not mapped)
            //      0    |    n     | copy pending (data not resident & not mapped)
            //      1    |    0     | eviction pending (data resident & mapped)
            //      1    |    n     | resident (data resident & mapped)

            // residency is set to Resident or NotResident by the notify thread
            // residency is read by process feedback thread, and set to transient states Evicting or Loading
            enum Residency
            {
                NotResident = 0, // b00
                Resident = 1,    // b01
                Evicting = 2,    // b10
                Loading = 3,     // b11
            };

            // called in UpdateMinMipMap()
            bool GetResident(UINT x, UINT y, UINT s) const { return (Residency::Resident == m_resident[s][y][x]); }

            // called by DataUploader during Notify* routines
            void SetResident(UINT x, UINT y, UINT s) { m_resident[s][y][x] = Residency::Resident; }
            void SetNotResident(UINT x, UINT y, UINT s) { m_resident[s][y][x] = Residency::NotResident; }
            // for debugging, called by DataUploader Notify* routines
            BYTE GetResidency(const D3D12_TILED_RESOURCE_COORDINATE& in_coord) const { return m_resident[in_coord.Subresource][in_coord.Y][in_coord.X]; }

            // called via TUM::ProcessFeedback() when queueing evictions and loads
            // redundant state represents that the heap index isn't reliable, as the resource is in the process of loading or evicting
            void SetLoading(UINT x, UINT y, UINT s) { m_resident[s][y][x] = Residency::Loading; }
            void SetEvicting(UINT x, UINT y, UINT s) { m_resident[s][y][x] = Residency::Evicting; }
            UINT32& GetHeapIndex(UINT x, UINT y, UINT s) { return m_heapIndices[s][y][x]; }

            // called via TUM::ProcessFeedback() in StreamingResource ProcessFeedback
            UINT32& GetRefCount(UINT x, UINT y, UINT s) { return m_refcounts[s][y][x]; }
            UINT32 GetRefCount(UINT x, UINT y, UINT s) const { return m_refcounts[s][y][x]; }

            bool GetResident(const D3D12_TILED_RESOURCE_COORDINATE& in_coord) const { return GetResident(in_coord.X, in_coord.Y, in_coord.Subresource); }
            void SetResident(const D3D12_TILED_RESOURCE_COORDINATE& in_coord) { SetResident(in_coord.X, in_coord.Y, in_coord.Subresource); }
            void SetNotResident(const D3D12_TILED_RESOURCE_COORDINATE& in_coord) { SetNotResident(in_coord.X, in_coord.Y, in_coord.Subresource); }
            void SetLoading(const D3D12_TILED_RESOURCE_COORDINATE& in_coord) { SetLoading(in_coord.X, in_coord.Y, in_coord.Subresource); }
            void SetEvicting(const D3D12_TILED_RESOURCE_COORDINATE& in_coord) { SetEvicting(in_coord.X, in_coord.Y, in_coord.Subresource); }

            UINT32& GetRefCount(const D3D12_TILED_RESOURCE_COORDINATE& in_coord) { return GetRefCount(in_coord.X, in_coord.Y, in_coord.Subresource); }
            UINT32 GetRefCount(const D3D12_TILED_RESOURCE_COORDINATE& in_coord) const { return GetRefCount(in_coord.X, in_coord.Y, in_coord.Subresource); }

            UINT32& GetHeapIndex(const D3D12_TILED_RESOURCE_COORDINATE& in_coord) { return GetHeapIndex(in_coord.X, in_coord.Y, in_coord.Subresource); }

            // checks refcount of bottom-most non-packed tile(s). If none are in use, we know nothing is resident.
            // used in UpdateMinMipMap()
            bool GetAnyRefCount();

            // return true if all bottom layer standard tiles are resident
            // Can accelerate UpdateMinMipMap()
            UINT8 GetMinResidentMip();

            // remove all mappings from a heap. useful when removing an object from a scene
            void FreeHeapAllocations(Streaming::Heap* in_pHeap);

            UINT GetWidth(UINT in_s) { return (UINT)m_resident[in_s][0].size(); }
            UINT GetHeight(UINT in_s) { return (UINT)m_resident[in_s].size(); }
        private:
            template<typename T> using TileRow = std::vector<T>;
            template<typename T> using TileY = std::vector<TileRow<T>>;
            template<typename T> using TileLayer = std::vector<TileY<T>>;

            TileLayer<BYTE> m_resident;
            TileLayer<UINT32> m_refcounts;
            TileLayer<UINT32> m_heapIndices;
        };
        TileMappingState m_tileMappingState;

        Streaming::TileUpdateManagerBase* m_pTileUpdateManager;

        std::vector<UINT> m_packedMipHeapIndices;

        // do not immediately decmap:
        // need to withhold until in-flight command buffers have completed
        class EvictionDelay
        {
        public:
            EvictionDelay(UINT in_numSwapBuffers);

            using MappingCoords = std::vector<D3D12_TILED_RESOURCE_COORDINATE>;
            void Append(D3D12_TILED_RESOURCE_COORDINATE in_coord) { m_mappings[0].push_back(in_coord); }
            MappingCoords& GetReadyToEvict() { return m_mappings.back(); }

            void NextFrame();
            void Clear();

            // drop pending evictions for tiles that now have non-zero refcount
            void Rescue(const TileMappingState& in_tileMappingState);
        private:
            std::vector<MappingCoords> m_mappings;
        };
        EvictionDelay m_pendingEvictions;

        std::vector<D3D12_TILED_RESOURCE_COORDINATE> m_pendingTileLoads;

        std::vector<BYTE> m_paddedPackedMips;

        //--------------------------------------------------------
        // for public interface
        //--------------------------------------------------------
        UINT m_residencyMapOffsetBase{ 0 };

        // used by QueueEviction()
        std::atomic<bool> m_setZeroRefCounts{ false };

        // minimum mip level referred to by this tile
        // this tile "holds references" to this mip level and all mips greater than this value
        // with a 16kx16k limit, DX will never see 255 mip levels. but, we want a byte so we can modify cache-coherently
        using TileReference = UINT8;
        std::vector<TileReference> m_tileReferences;
        UINT m_tileReferencesWidth;  // function of resource tiling
        UINT m_tileReferencesHeight; // function of resource tiling

        UINT8 m_maxMip;
        std::vector<BYTE, Streaming::AlignedAllocator<BYTE>> m_minMipMap; // local version of min mip map, rectified in UpdateMinMipMap()

    private:
        // non-packed mip copy complete notification
        std::atomic<bool> m_tileResidencyChanged{ false };

        // drop pending loads that are no longer relevant
        void AbandonPending();

        // index to next min-mip feedback resolve target
        UINT m_readbackIndex;

        // if feedback is queued, it is ready to use after the render fence has reached this value
        // support having a feedback queued every frame (num swap buffers)
        struct QueuedFeedback
        {
            UINT64 m_renderFenceForFeedback{ UINT_MAX };
            std::atomic<bool> m_feedbackQueued{ false }; // written by render thread, read by UpdateFeedback() thread
        };
        std::vector<QueuedFeedback> m_queuedFeedback;

        // update internal mapping and refcounts for each tile
        void SetMinMip(UINT8 in_current, UINT in_x, UINT in_y, UINT in_s);

        // AddRef, which requires allocation, might fail
        void AddTileRef(UINT in_x, UINT in_y, UINT in_s);

        // DecRef may decline
        void DecTileRef(UINT in_x, UINT in_y, UINT in_s);

        // only push evictions to DataUploader once per rendered frame (i.e. "on the next frame")
        void QueuePendingTileEvictions(Streaming::UpdateList* out_pUpdateList);

        void QueuePendingTileLoads(Streaming::UpdateList* out_pUpdateList); // returns # tiles queued

        void PadPackedMips(ID3D12Device* in_pDevice);

        // used by QueueEviction()
        bool m_refCountsZero{ true };
    };
}
