// let wasm = require("../wasm/build/fuzzy-bools");
import wasm from "./fuzzy-bools.js";

console.log(wasm);

function GetVertexArray(heap, ptr, size) {
    return getSubArray(heap, ptr / 2, size);
}

function GetIndexArray(heap, ptr, size) {
    return getSubArray(heap, ptr, size);
}

function getSubArray(heap, startPtr, size) {
    return heap.subarray(startPtr / 4, startPtr / 4 + size);
}

export function DoBool(wasmModule, vertsA, indicesA, vertsB, indicesB)
{
    let vsize_a = vertsA.length;
    let isize_a = indicesA.length;
    let vsize_b = vertsB.length;
    let isize_b = indicesB.length;

    let floatHeap = wasmModule.HEAPF64;
    let uintHeap = wasmModule.HEAPU32;

    let allocptrs = wasmModule.Alloc(vsize_a,isize_a,vsize_b,isize_b);

    console.log(allocptrs);

    let vptr_a = GetVertexArray(floatHeap, allocptrs.vptr_a, vsize_a);
    let iptr_a = GetIndexArray(uintHeap, allocptrs.iptr_a, isize_a);
    let vptr_b = GetVertexArray(floatHeap, allocptrs.vptr_b, vsize_b);
    let iptr_b = GetIndexArray(uintHeap, allocptrs.iptr_b, isize_b);

    vptr_a.set(vertsA);
    iptr_a.set(indicesA);
    vptr_b.set(vertsB);
    iptr_b.set(indicesB);

    console.log(vptr_a);
    console.log(iptr_a);
    console.log(vptr_b);
    console.log(iptr_b);

    let ptrs = wasmModule.DoSubtract();

    let result = {};
    
    floatHeap = wasmModule.HEAPF64;
    uintHeap = wasmModule.HEAPU32;

    result.verts = GetVertexArray(floatHeap, ptrs.vptr, ptrs.numVertices).slice(0);
    result.indices = GetIndexArray(uintHeap, ptrs.iptr, ptrs.numIndices).slice(0);

    console.log("done");

    return result;
}


export async function GetModule()
{
    return await wasm();
}