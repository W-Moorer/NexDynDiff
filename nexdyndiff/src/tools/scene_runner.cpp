#include <iostream>
#include <nexdyndiff.h>
#include "../scene/SceneRunner.h"

void PrintUsage(const char* program_name)
{
    std::cout << "Usage: " << program_name << " <scene_file> [options]\n\n";
    std::cout << "Arguments:\n";
    std::cout << "  <scene_file>           Path to scene file (.json or .xml)\n\n";
    std::cout << "Options:\n";
    std::cout << "  --no-cache             Disable scene caching\n";
    std::cout << "  --no-validation        Disable scene validation\n";
    std::cout << "  --quiet                Suppress verbose output\n";
    std::cout << "  -h, --help             Show this help message\n";
    std::cout << "\nExamples:\n";
    std::cout << "  " << program_name << " scenes/hanging_cloth.json\n";
    std::cout << "  " << program_name << " scenes/my_scene.xml --no-cache\n";
}

int main(int argc, char* argv[])
{
    if (argc < 2) {
        PrintUsage(argv[0]);
        return 1;
    }

    std::filesystem::path scene_file = argv[1];

    if (scene_file == "-h" || scene_file == "--help") {
        PrintUsage(argv[0]);
        return 0;
    }

    nexdyndiff::scene::RunOptions options;

    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--no-cache") {
            options.use_cache = false;
        }
        else if (arg == "--no-validation") {
            options.validate = false;
        }
        else if (arg == "--quiet") {
            options.verbose = false;
        }
        else {
            std::cerr << "Unknown option: " << arg << "\n";
            PrintUsage(argv[0]);
            return 1;
        }
    }

    return nexdyndiff::scene::SceneRunner::Run(scene_file, options);
}
