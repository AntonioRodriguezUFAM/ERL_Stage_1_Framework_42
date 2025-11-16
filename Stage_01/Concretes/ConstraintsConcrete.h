#pragma once
#include "../Interfaces/IConstraints.h"
#include <iostream>

class ConstraintsConcrete : public IConstraints {
public:
    void applyConstraints() override {
        std::cout << "\n  [ConstraintsConcrete] Applying Constraints...\n";
    }
};
