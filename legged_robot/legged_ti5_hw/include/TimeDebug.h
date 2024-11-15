#include <chrono>

// 定义DEBUG宏以启用调试输出
#ifdef TIMEPRINT
    #define DEBUG_PRINT(x) std::cout << x << std::endl
    #define DEBUG_TIMER_START(timer) timer.start()
    #define DEBUG_TIMER_DURATION(timer, message) \
        timer.stop(); \
        std::cout << message << " took " << timer.elapsedMilliseconds() << " milliseconds." << std::endl; \
        timer.start()
#else
    #define DEBUG_PRINT(x) do {} while (0)
    #define DEBUG_TIMER_START(timer) do {} while (0)
    #define DEBUG_TIMER_DURATION(timer, message) do {} while (0)
#endif

class Timer {
public:
    void start() {
        start_time_ = std::chrono::high_resolution_clock::now();
    }

    void stop() {
        end_time_ = std::chrono::high_resolution_clock::now();
    }

    double elapsedMilliseconds() const {
        std::chrono::duration<double, std::milli> duration = end_time_ - start_time_;
        return duration.count();
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
    std::chrono::time_point<std::chrono::high_resolution_clock> end_time_;
};
