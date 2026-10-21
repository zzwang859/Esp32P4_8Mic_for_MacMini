"""Exercise generated ESP-DL arena ownership/reuse with allocation failures."""
import argparse
from pathlib import Path
import subprocess
import tempfile


def block(text, marker):
    start = text.index(marker)
    opening = text.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[start:end]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=Path("build/esp-idf/src"))
    parser.add_argument("--cxx", nargs="+", required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    context = (args.build_dir / "kws_dl_model_context.cpp").read_text(encoding="utf-8")
    planner = (args.build_dir / "kws_dl_memory_manager_greedy.cpp").read_text(encoding="utf-8")
    header = (root / "managed_components/espressif__esp-dl/dl/model/include/dl_model_context.hpp").read_text(encoding="utf-8")
    allocation = block(context, "bool ModelContext::root_alloc(")
    free = block(header, "void root_free()")
    reset = block(header, "void root_reset()")
    prepare = block(planner, "if (kws_model_tensor_arena_requested())")
    source = r'''
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <map>
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define MALLOC_CAP_SPIRAM 1
#define MALLOC_CAP_INTERNAL 2
static std::map<void *, size_t> live;
static unsigned internal_allocs, psram_allocs;
static bool fail_internal, fail_psram;
size_t heap_caps_get_largest_free_block(unsigned) { return 1000000; }
void tracked_free(void *p) { assert(live.erase(p) == 1); std::free(p); }
namespace dl {
namespace tool {
void *calloc_aligned(size_t n, size_t size, unsigned caps) {
    if ((caps == MALLOC_CAP_INTERNAL && fail_internal) ||
        (caps == MALLOC_CAP_SPIRAM && fail_psram)) return nullptr;
    if (caps == MALLOC_CAP_INTERNAL) ++internal_allocs; else ++psram_allocs;
    void *p = std::calloc(n, size); assert(p); live[p] = n * size; return p;
}
}
class ModelContext {
public:
    void *m_internal_root = nullptr, *m_psram_root = nullptr;
    int m_internal_size = 0, m_psram_size = 0;
    bool root_alloc(size_t, size_t, int = 16);
#define free tracked_free
''' + free + "\n" + reset + r'''
#undef free
    ~ModelContext() { root_free(); }
};
''' + allocation + r'''
}
int kws_model_tensor_budget() { return 59392; }
bool requested = true;
bool kws_model_tensor_arena_requested() { return requested; }
class Planner {
public:
    size_t max_internal_size;
    int alignment = 16;
    void prepare(dl::ModelContext *context) {
''' + prepare + r'''
    }
};
int main() {
    {
        dl::ModelContext context;
        Planner planner{0}; planner.prepare(&context);
        assert(planner.max_internal_size == 59392 && internal_allocs == 1);
        void *arena = context.m_internal_root;
        std::memset(arena, 0x5a, 59392);
        assert(context.root_alloc(59392, 8192));
        assert(context.m_internal_root == arena && internal_allocs == 1 && psram_allocs == 1);
        assert(static_cast<unsigned char *>(arena)[59391] == 0x5a);
        context.root_reset();
        assert(static_cast<unsigned char *>(arena)[59391] == 0);
        assert(!context.root_alloc(60000, 8192)); // Never resize a live arena.
        context.root_free(); context.root_free(); // Idempotent, no double free.
        assert(live.empty());
    }
    {
        dl::ModelContext context;
        requested = false;
        Planner normal{32768}; normal.prepare(&context);
        assert(!context.m_internal_root && normal.max_internal_size == 32768);
        fail_internal = true;
        requested = true;
        Planner fallback{0}; fallback.prepare(&context);
        assert(fallback.max_internal_size == 0 && !context.m_internal_root);
        fail_internal = false;
        assert(context.root_alloc(0, 57904)); // Fallback plans entirely in PSRAM.
    }
    assert(live.empty());
    {
        dl::ModelContext context;
        Planner planner{0}; planner.prepare(&context);
        fail_psram = true;
        assert(!context.root_alloc(59392, 8192));
        assert(context.m_internal_root && !context.m_psram_root);
        fail_psram = false;
    } // Failed final allocation still frees the pre-reserved arena once.
    assert(live.empty());
}
'''
    with tempfile.TemporaryDirectory(prefix="kws-arena-") as directory:
        folder = Path(directory)
        (folder / "test.cpp").write_text(source, encoding="utf-8")
        executable = folder / "arena-test.exe"
        subprocess.run(args.cxx + ["-std=c++17", "-Wall", "-Wextra", str(folder / "test.cpp"),
                       "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
    print("Generated ESP-DL arena ownership, fallback and cleanup: passed")


if __name__ == "__main__":
    main()
