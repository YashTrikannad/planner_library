//
// Created by yash on 8/4/19.
//

#include "random2d_map_generator.h"

#include <iostream>

template <typename ElementType>
void print_2d_vector(const std::vector<std::vector<ElementType>>& container)
{
    for(const auto& vector:container)
    {
        for(const auto& element:vector)
        {
            std::cout << element << " ";
        }
        std::cout << "\n";
    }
}

int main()
{
    map::MapGenerator<int, 0, 1> map_generator;

    map_generator.generate_map(10, 10, 4);

    const auto map = map_generator.get_map();

    print_2d_vector(map);

    return 0;
}