// SystemProfilingConcrete.h

#pragma once
#include <iostream>

#include "../Interfaces/ISystemProfiling.h"  // If inside "Stage_01/Interfaces/"
#include "../SharedStructures/jetsonNanoInfo.h"  // Ensure JetsonNanoInfo is visible
#include "../Stage_01/Concretes/AlgorithmConcrete.h"
#include "../Stage_01/Concretes/SoCConcrete.h"
#include "../Stage_01/Concretes/DataConcrete.h"
#include <tuple>

class SystemProfilingConcrete : public ISystemProfiling {
public:
    // Constructor using AlgorithmConcrete, SoCConcrete, and DataConcrete
    SystemProfilingConcrete(IAlgorithm& algo, ISoC& soc, IData& data, SystemCaptureFactory& captureFactory) 
        : algo_(algo), soc_(soc), data_(data), captureFactory_(captureFactory) {} 

    // Constructor using SystemCaptureFactory
    SystemProfilingConcrete(SystemCaptureFactory& captureFactory) 
        : captureFactory_(captureFactory), 
          algo_(*captureFactory.getAlgorithm()), 
          soc_(*captureFactory.getSoC()), 
          data_(*captureFactory.getCamera(0)) // Assuming you want metrics from the first camera
    {} 

    JetsonNanoInfo getSoCMetrics() const override { 
        return soc_.getPerformance(); 
    }

    std::tuple<double, int> getCameraMetrics(int cameraIndex) const override { 
        // Assuming captureFactory has access to camera metrics
        return captureFactory_.getCameraMetrics(cameraIndex); 
    }

    std::tuple<double, double> getAlgorithmMetrics() const override { 
        return std::make_tuple(algo_.getFps(),algo_.getAverageProcTime());
    }
private:
    IAlgorithm& algo_;
    ISoC& soc_;
    IData& data_;

    SystemCaptureFactory& captureFactory_; 
};






// #pragma once
// #include <iostream>



// #include "../Interfaces/ISystemProfiling.h"  // If inside "Stage_01/Interfaces/"
// #include "../Stage_01/Factories/SystemCaptureFactory.h" 
// #include "../SharedStructures/jetsonNanoInfo.h"  // Ensure JetsonNanoInfo is visible
// #include "../Stage_01/Concretes/AlgorithmConcrete.h"
// #include "../Stage_01/Concretes/SoCConcrete.h"
// #include "../Stage_01/Concretes/DataConcrete.h"
// #include <tuple>

// class SystemProfilingConcrete : public ISystemProfiling {
// public:
//  // Constructor 
//     SystemProfilingConcrete(AlgorithmConcrete& algo, SoCConcrete& soc, DataConcrete& data) 
//         : algo_(algo), soc_(soc), data_(data) {} 

//     // Constructor using SystemCaptureFactory
//     SystemProfilingConcrete(SystemCaptureFactory& captureFactory) 
//         : captureFactory_(captureFactory) {} 


//     JetsonNanoInfo getSoCMetrics() const override { 
//        // Implement logic to get SoC metrics 
//        //JetsonNanoInfo socInfo; 
//        // ... populate socInfo with data ...
//        return soc_.getPerformance(); 
//     }

//     std::tuple<double, int> getCameraMetrics(int cameraIndex) const override { 
//         // Implement logic to get camera metrics for the specified camera 
//         //double fps = 0.0;
//         //int queueSize = 0;
//         // ... calculate fps and queueSize ...
//         return { data_.getLastFPS(), data_.getQueueSize() };
        
//     }

//     std::tuple<double, double> getAlgorithmMetrics() const override { 
//         // Implement logic to get algorithm metrics 
//         //double fps = 0.0;
//         //double procTime = 0.0;
//         // ... calculate fps and procTime ...
//        return std::make_tuple(algo_.getFps(),algo_.getAverageProcTime());
//     }
// private:
//     AlgorithmConcrete& algo_;
//     SoCConcrete& soc_;
//     DataConcrete& data_;

//     SystemCaptureFactory& captureFactory_; 

// };
