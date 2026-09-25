#pragma once

#include <rmcs_executor/component.hpp>
#include <stdexcept>
#include <string>

// Standalone executor fixture: uses the same interface binding as the real executor,
// but never creates a hardware component or starts its update thread.
namespace rmcs_executor {
class Executor {
public:
    template <typename T>
    static void bind(Component& component, const std::string& name, T& value) {
        for (auto& input : component.input_list_) {
            if (input.name == name && input.type == typeid(T)) {
                input.bind(input.binding, &value);
                return;
            }
        }
        throw std::runtime_error("missing input: " + name);
    }
    template <typename T>
    static T& output(Component& component, const std::string& name) {
        for (auto& output : component.output_list_)
            if (output.name == name && output.type == typeid(T))
                return *static_cast<T*>(output.binding);
        throw std::runtime_error("missing output: " + name);
    }
};
}
