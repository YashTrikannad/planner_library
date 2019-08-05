//
// Created by yash on 8/4/19.
//

#pragma once

#include <iostream>
#include <vector>

namespace pfl::common
{

template<typename ElementType>
void print_2d_vector(const std::vector<std::vector<ElementType>> &container)
{
    for (const auto &vector:container)
    {
        for (const auto &element:vector)
        {
            std::cout << element << " ";
        }
        std::cout << "\n";
    }
}

} // namespace pfl::common