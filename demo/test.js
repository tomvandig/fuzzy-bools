let wasm = require("../wasm/build/fuzzy-bools");


function GetVertexArray(heap, ptr, size) {
    return getSubArray(heap, ptr / 2, size);
}

function GetIndexArray(heap, ptr, size) {
    return getSubArray(heap, ptr, size);
}

function getSubArray(heap, startPtr, size) {
    return heap.subarray(startPtr / 4, startPtr / 4 + size);
}

function DoBool(wasmModule)
{
    let vsize_a = 4;
    let isize_a = 3;
    let vsize_b = 2;
    let isize_b = 1;

    let floatHeap = wasmModule.HEAPF64;
    let uintHeap = wasmModule.HEAPU32;

    let allocptrs = wasmModule.Alloc(vsize_a,isize_a,vsize_b,isize_b);

    console.log(allocptrs);

    let vptr_a = GetVertexArray(floatHeap, allocptrs.vptr_a, vsize_a);
    let iptr_a = GetIndexArray(uintHeap, allocptrs.iptr_a, isize_a);
    let vptr_b = GetVertexArray(floatHeap, allocptrs.vptr_b, vsize_b);
    let iptr_b = GetIndexArray(uintHeap, allocptrs.iptr_b, isize_b);

    console.log(vptr_a);
    console.log(iptr_a);
    console.log(vptr_b);
    console.log(iptr_b);

    vptr_a[1] = 5.2;
    iptr_a[1] = 13;

    let ptrs = wasmModule.DoSubtract();

    let result = {};
    
    result.verts = GetVertexArray(floatHeap, ptrs.vptr, ptrs.numVertices).slice(0);
    result.indices = GetIndexArray(uintHeap, ptrs.iptr, ptrs.numIndices).slice(0);

    return result;
}

wasm().then((mod) => {
    console.log(mod);
    console.log(mod.GetVersion())

    let result = DoBool(mod);

    console.log(result.verts);
    console.log(result.indices);
});