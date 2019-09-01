//
// planning library
// Created by yash on 9/1/19.
//

#include "../common/data_types.h"
#include "../graph/types/CostGraph.h"
#include "../graph/types/VectorGraph.h"
#include "../map_generation/random2d_map_generator.h"

#include <benchmark/benchmark.h>
#include <astar.h>

static void BM_MapGeneration(benchmark::State& state) {
    std::set<int> data;
    for (auto _ : state)
    {
        pl::map::ConstraintMapGenerator<size_t, 0, 1> map_generator;
        map_generator.generate_map(state.range(0), state.range(1), state.range(2));
        auto map = map_generator.get_map();
    }
}
BENCHMARK(BM_MapGeneration)
        ->Args({100, 100, 5})
        ->Args({200, 200, 5})
        ->Args({400, 400, 5})
        ->Args({800, 800, 5})
        ->Args({1600, 1600, 5})
        ->Args({100, 100, 10})
        ->Args({200, 200, 10})
        ->Args({400, 400, 10})
        ->Args({800, 800, 10})
        ->Args({1600, 1600, 10});

static void BM_VectorGraphCreation(benchmark::State& state) {
    std::set<int> data;
    for (auto _ : state)
    {
        pl::map::ConstraintMapGenerator<size_t, 0, 1> map_generator;
        map_generator.generate_map(state.range(0), state.range(1), state.range(2));
        auto map = map_generator.get_map();
        auto vector_graph = pl::graph::cost_graph<pl::graph::graph, std::vector<std::vector<size_t>>, size_t>{map};
    }
}
BENCHMARK(BM_VectorGraphCreation)
        ->Args({100, 100, 5})
        ->Args({200, 200, 5})
        ->Args({400, 400, 5})
        ->Args({800, 800, 5})
        ->Args({1600, 1600, 5})
        ->Args({100, 100, 10})
        ->Args({200, 200, 10})
        ->Args({400, 400, 10})
        ->Args({800, 800, 10})
        ->Args({1600, 1600, 10});

static void BM_VectorGraphAstarSearch(benchmark::State& state) {
    std::set<int> data;
    for (auto _ : state)
    {
        pl::map::ConstraintMapGenerator<size_t, 0, 1> map_generator;
        map_generator.generate_map(state.range(0), state.range(1), state.range(2));
        auto map = map_generator.get_map();
        auto vector_graph = pl::graph::cost_graph<pl::graph::graph, std::vector<std::vector<size_t>>, size_t>{map};
        pl::algorithms::astar<pl::graph::cost_graph<pl::graph::graph, std::vector<std::vector<size_t>>, size_t>,
                std::vector<pl::common::NodeIndex2d>, pl::common::NodeIndex2d>
                eigen_planner(&vector_graph);
        eigen_planner.find_path<8>({0, 0}, {state.range(0)/2, state.range(1)/2});
        const auto eigen_path = eigen_planner.get_path();
    }
}
BENCHMARK(BM_VectorGraphAstarSearch)
        ->Args({100, 100, 5})
        ->Args({200, 200, 5})
        ->Args({400, 400, 5})
        ->Args({800, 800, 5})
        ->Args({1600, 1600, 5})
        ->Args({100, 100, 10})
        ->Args({200, 200, 10})
        ->Args({400, 400, 10})
        ->Args({800, 800, 10})
        ->Args({1600, 1600, 10});

BENCHMARK_MAIN();
