#pragma once
#include <alnslike/core/post_processor.hpp>
#include <alnslike/core/search_step.hpp>
#include <alnslike/core/solution.hpp>
#include <alnslike/core/solution_generator.hpp>
#include <alnslike/core/types.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <stdexcept>
#include <utility>

namespace alnslike::core {

constexpr std::size_t kDefaultMaxIteration = 1'000'000'000;
constexpr std::size_t kDefaultMaxIterationWithoutImprovement = 100'000'000;
constexpr std::size_t kDefaultTimeout = 100'000'000;
constexpr std::size_t kDefaultTimeCheckMask = 1024 - 1;

struct SolverConfig {
    std::size_t max_iterations{kDefaultMaxIteration};
    std::size_t max_iterations_without_improvement{kDefaultMaxIterationWithoutImprovement};
    std::chrono::duration<double> timeout{kDefaultTimeout};
    uint64_t seed{0};  // 0 = use std::random_device

    std::size_t time_check_mask{kDefaultTimeCheckMask};
};

using StepObserver = std::function<void(const SolverState&, const StepResult&, const Solution&)>;

class SolverBuilder;

class Solver final {
    friend class SolverBuilder;

    explicit Solver(std::unique_ptr<Solution> initial_solution, std::unique_ptr<SolutionGenerator> generator,
                    std::unique_ptr<SearchStep> search_step, std::unique_ptr<AcceptanceCriterion> acceptance_criterion,
                    std::unique_ptr<MoveCache> move_cache, SolverConfig config, StepObserver observer,
                    std::unique_ptr<PostProcessor> post_processor)
        : config_(config),
          actual_seed_(config_.seed == 0 ? std::random_device{}() : config_.seed),
          rng_(actual_seed_),
          initial_solution_(std::move(initial_solution)),
          solution_generator_(std::move(generator)),
          search_step_(std::move(search_step)),
          acceptance_criterion_(std::move(acceptance_criterion)),
          move_cache_(std::move(move_cache)),
          observer_(std::move(observer)),
          post_processor_(std::move(post_processor)),
          best_solution_(nullptr) {}

public:
    ~Solver() = default;
    Solver(Solver&&) noexcept = default;
    Solver& operator=(Solver&&) = default;
    Solver(const Solver&) = delete;
    Solver& operator=(const Solver&) = delete;

    SolverState Run() {
        InitializeRun();

        while (ShouldContinue()) {
            ExecuteIteration();
            ++state_.current_iteration;
        }

        ExecutePostProcessing();

        state_.elapsed_time = std::chrono::steady_clock::now() - start_time_;
        return state_;
    }

    [[nodiscard]] const Solution& BestSolution() const noexcept { return *best_solution_; }
    [[nodiscard]] const SolverState& State() const noexcept { return state_; }
    [[nodiscard]] uint64_t Seed() const noexcept { return actual_seed_; }

private:
    SolverConfig config_;
    uint64_t actual_seed_;
    Rng rng_;

    std::unique_ptr<Solution> initial_solution_;
    std::unique_ptr<SolutionGenerator> solution_generator_;
    std::unique_ptr<SearchStep> search_step_;
    std::unique_ptr<AcceptanceCriterion> acceptance_criterion_;
    std::unique_ptr<MoveCache> move_cache_;
    StepObserver observer_;
    std::unique_ptr<PostProcessor> post_processor_;

    std::unique_ptr<Solution> current_solution_;
    std::unique_ptr<Solution> best_solution_;
    SolverState state_;

    std::chrono::steady_clock::time_point start_time_;
    bool is_solved_{false};

    void InitializeRun() noexcept {
        if (initial_solution_) {
            current_solution_ = initial_solution_->Clone();
        } else {
            current_solution_ = solution_generator_->CreateAndGenerate(rng_);
        }

        best_solution_ = current_solution_->Clone();
        is_solved_ = best_solution_->IsSolved();

        state_ = SolverState{};
        state_.current_best_cost = best_solution_->GetCost();

        start_time_ = std::chrono::steady_clock::now();
    }

    [[nodiscard]] bool ShouldContinue() noexcept {
        if (is_solved_) {
            return false;
        }

        if (state_.current_iteration >= config_.max_iterations) {
            return false;
        }

        if (state_.iterations_without_improvement >= config_.max_iterations_without_improvement) {
            return false;
        }

        if ((state_.current_iteration & config_.time_check_mask) == 0) {
            state_.elapsed_time = std::chrono::steady_clock::now() - start_time_;
            if (state_.elapsed_time >= config_.timeout) {
                return false;
            }
        }

        return true;
    }

    void ExecuteIteration() noexcept {
        const StepResult step_result =
            search_step_->Execute(*current_solution_, *move_cache_, *acceptance_criterion_, state_, rng_);

        if (step_result.is_new_best) {
            best_solution_->CopyFrom(*current_solution_);
            state_.current_best_cost = current_solution_->GetCost();
            state_.iterations_without_improvement = 0;

            if (best_solution_->IsSolved()) {
                is_solved_ = true;
            }
        } else {
            ++state_.iterations_without_improvement;
        }

        if (observer_) {
            observer_(state_, step_result, *current_solution_);
        }
    }

    void ExecutePostProcessing() noexcept {
        if (post_processor_ && best_solution_) {
            post_processor_->Optimize(*best_solution_, rng_);
            state_.current_best_cost = best_solution_->GetCost();
        }
    }
};

class SolverBuilder final {
public:
    SolverBuilder& SetMaxIterations(std::size_t max_iterations) {
        config_.max_iterations = max_iterations;
        return *this;
    }

    SolverBuilder& SetMaxIterationsWithoutImprovement(std::size_t max_iterations) {
        config_.max_iterations_without_improvement = max_iterations;
        return *this;
    }

    SolverBuilder& SetTimeout(std::chrono::duration<double> timeout) {
        if (timeout.count() <= 0) {
            throw std::invalid_argument("SolverBuilder: таймаут должен быть > 0.");
        }
        config_.timeout = timeout;
        return *this;
    }

    SolverBuilder& SetSeed(uint64_t seed) {
        config_.seed = seed;
        return *this;
    }

    SolverBuilder& SetTimeCheckFrequencyMask(std::size_t mask) {
        if ((mask & (mask + 1)) != 0) {
            throw std::invalid_argument("SolverBuilder: маска проверки времени должна быть (2^N - 1).");
        }
        config_.time_check_mask = mask;
        return *this;
    }

    SolverBuilder& SetInitialSolution(std::unique_ptr<Solution> initial_solution) {
        if (!initial_solution) {
            throw std::invalid_argument("SolverBuilder: передано пустое решение.");
        }
        initial_solution_ = std::move(initial_solution);
        return *this;
    }

    SolverBuilder& SetGenerator(std::unique_ptr<SolutionGenerator> generator) {
        if (!generator) {
            throw std::invalid_argument("SolverBuilder: передан пустой генератор.");
        }
        solution_generator_ = std::move(generator);
        return *this;
    }

    SolverBuilder& SetSearchStep(std::unique_ptr<SearchStep> search_step) {
        if (!search_step) {
            throw std::invalid_argument("SolverBuilder: передан пустой SearchStep.");
        };
        search_step_ = std::move(search_step);
        return *this;
    }

    SolverBuilder& SetAcceptanceCriterion(std::unique_ptr<AcceptanceCriterion> acceptance_criterion) {
        if (!acceptance_criterion) {
            throw std::invalid_argument("SolverBuilder: передан пустой критерий принятия.");
        }
        acceptance_criterion_ = std::move(acceptance_criterion);
        return *this;
    }

    SolverBuilder& SetMoveCache(std::unique_ptr<MoveCache> move_cache) {
        move_cache_ = std::move(move_cache);
        return *this;
    }

    SolverBuilder& SetObserver(StepObserver observer) {
        observer_ = std::move(observer);
        return *this;
    }

    SolverBuilder& SetPostProcessor(std::unique_ptr<PostProcessor> post_processor) {
        post_processor_ = std::move(post_processor);
        return *this;
    }

    [[nodiscard]] std::unique_ptr<Solver> Build() {
        if (!solution_generator_ && !initial_solution_) {
            throw std::logic_error("SolverBuilder: не задан генератор или начальное решение.");
        }
        if (!search_step_) {
            throw std::logic_error("SolverBuilder: не задана стратегия шага поиска.");
        }
        if (!acceptance_criterion_) {
            throw std::logic_error("SolverBuilder: не задан критерий принятия.");
        }

        return std::unique_ptr<Solver>(new Solver(std::move(initial_solution_), std::move(solution_generator_),
                                                  std::move(search_step_), std::move(acceptance_criterion_),
                                                  std::move(move_cache_), config_, std::move(observer_),
                                                  std::move(post_processor_)));
    }

private:
    SolverConfig config_{};
    std::unique_ptr<Solution> initial_solution_;
    std::unique_ptr<SolutionGenerator> solution_generator_;
    std::unique_ptr<SearchStep> search_step_;
    std::unique_ptr<AcceptanceCriterion> acceptance_criterion_;
    std::unique_ptr<MoveCache> move_cache_;
    StepObserver observer_;
    std::unique_ptr<PostProcessor> post_processor_;
};

}  // namespace alnslike::core