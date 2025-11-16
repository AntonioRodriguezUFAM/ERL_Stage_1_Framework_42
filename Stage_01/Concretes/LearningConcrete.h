#pragma once
#include "../Interfaces/ILearning.h"
#include <iostream>

class LearningConcrete : public ILearning {
public:
    void learnPattern() override {
        std::cout << "\n  [LearningConcrete] Learning Pattern...\n";
    }

    void adapt() override {
        std::cout << "\n  [LearningConcrete] Adapting...\n";
    }
};
