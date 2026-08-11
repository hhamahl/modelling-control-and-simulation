#ifndef GENETIC_ALGORITHMS_H_
#define GENETIC_ALGORITHMS_H_

#include <bitset>
#include <random>
#include <vector>
#include <cmath>
#include <cassert>


namespace genetic_algorithms {

template <std::size_t N> 
class Individual {
public:
    Individual() = default;

    Individual(std::size_t nvariables, 
               std::default_random_engine& el, 
               std::uniform_int_distribution<int>& uniform_dist);// N=10


    unsigned long long GrayToBinary(const std::bitset<N>& gray);

    std::vector<double> DecodeGrayChromosome(double max);

    void Mutate(double p_mutate, 
                std::default_random_engine& el, 
                std::uniform_real_distribution<double>& uniform_dist);

    std::vector<std::bitset<N>> chromosome;
    size_t n; //number of variables
    double fitness{0.0};
};

template <std::size_t N>
class Population {
    public:
        Population() = default;
        Population(size_t size, size_t nvariables);
        
        std::pair<int, int> Tournament(double p_tour,
                std::default_random_engine& el,
                std::uniform_int_distribution<int>& selection_dist,
                std::uniform_real_distribution<double>& winner_dist
                );

        std::pair<Individual<N>, Individual<N>> CrossOver(std::pair<int, int>& selection, double p_cross,
                            std::default_random_engine& el,
                            std::uniform_real_distribution<double>& cross_or_not,
                            std::uniform_int_distribution<int>& cross_point);

        
        std::vector<Individual<N>> individuals;
        size_t n; //population size


};

} // genetic_algorithms


namespace genetic_algorithms {

template <std::size_t N>
Individual<N>::Individual(size_t nvariables,
                       std::default_random_engine& el,
                       std::uniform_int_distribution<int>& uniform_dist)
{

    n = nvariables;
    chromosome.resize(n);

    for(size_t i = 0; i < n; i++) {
        for(size_t j = 0;  j < N; j++) {
            chromosome[i][j] = uniform_dist(el);
        }
    }

}

template <std::size_t N>
unsigned long long Individual<N>::GrayToBinary(const std::bitset<N>& gray)
{

    std::bitset<N> binary;

    // MSB remains the same
    binary[N-1] = gray[N-1];

    // Each subsequent bit is the XOR of the current Gray bit
    // and the previous converted binary bit
    for (int i = static_cast<int>(N) - 2; i >= 0; --i) {
        binary[i] = gray[i] ^ binary[i + 1];
    }
    return binary.to_ullong();

}

template <std::size_t N>
std::vector<double> Individual<N>::DecodeGrayChromosome(double max)
{
    // formula from book.
        std::vector<double> decoded(n);
        for(size_t i = 0; i < n; i++) {
            double y = static_cast<double>(GrayToBinary(chromosome[i]));
            decoded[i] = -max + (2*y*max)*std::pow(2, 
                    -static_cast<double>(N))/(1-std::pow(2, -static_cast<double>(N)));
        }

        return decoded;
}

template <std::size_t N>
void Individual<N>::Mutate(double p_mutate,
        std::default_random_engine& el,
        std::uniform_real_distribution<double>& uniform_dist)
{
    for (auto& gene : chromosome) { // <-- Added & here
        for (size_t i = 0; i < N; i++) {
            if (p_mutate > uniform_dist(el)) {
                gene[i].flip();
            }
        }
    }
}

template <std::size_t N>
Population<N>::Population(size_t size, size_t nvariables)
{
    n = size;
    // Seed with a real random value, if available
    std::random_device r;
    std::default_random_engine el(r());
    std::uniform_int_distribution<int> uniform_dist(0, 1);

    for(size_t i = 0; i < n; i++) {
        individuals.push_back(Individual<N>(nvariables, el, uniform_dist));
    }
    individuals.shrink_to_fit();
}

template<std::size_t N>
std::pair<int, int> Population<N>::Tournament(double p_tour,
                std::default_random_engine& el,
                std::uniform_int_distribution<int>& selection_dist,
                std::uniform_real_distribution<double>& winner_dist
                )
{

    std::pair<int, int> selection;

    // uniform_int_distr(0, N-1) to select two indicies(individuals)
    int i1 = selection_dist(el);
    int i2 = selection_dist(el);

    // uniform_real_distr(0,1) to select winner.
    if(winner_dist(el) < p_tour) {
        selection.first = individuals[i1].fitness > individuals[i2].fitness ? i1 : i2;
    } else {
        selection.first = individuals[i1].fitness <=  individuals[i2].fitness ? i1 : i2;
    }

    i1 = selection_dist(el);
    i2 = selection_dist(el);

    // uniform_real_distr(0,1) to select winner.
    if(winner_dist(el) < p_tour) {
        selection.second = individuals[i1].fitness >  individuals[i2].fitness ? i1 : i2;
    } else {
        selection.second = individuals[i1].fitness <= individuals[i2].fitness ? i1 : i2;
    }

    return selection;
}

template <std::size_t N>
std::pair<Individual<N>, Individual<N>> Population<N>::CrossOver(std::pair<int, int>& selection, double p_cross,
                            std::default_random_engine& el,
                            std::uniform_real_distribution<double>& cross_or_not,
                            std::uniform_int_distribution<int>& cross_point)
{
    std::pair<Individual<N>, Individual<N>> pair;

    if(p_cross < cross_or_not(el)){
        pair.first = individuals[selection.first];
        pair.second = individuals[selection.second];
        return pair;
    }

    int cross = cross_point(el);
    int variables = static_cast<int>(individuals[selection.first].n);
    if(cross == 0 || cross ==  N*variables) {
        pair.first = individuals[selection.first];
        pair.second = individuals[selection.second];
        return pair;
    }

    std::vector<std::bitset<N>> tmp1 = individuals[selection.first].chromosome;
    std::vector<std::bitset<N>> tmp2 = individuals[selection.second].chromosome;
    int count = 0;
    for(int row = 0; row < variables; row++) {
        for(int col = 0; col < N; col++) {
            if(count >= cross) {
                tmp1[row][col] =
                    individuals[selection.second].chromosome[row][col];
                tmp2[row][col] = individuals[selection.first].chromosome[row][col];
            }
            count++;
        }
    }
    pair.first.chromosome = tmp1;
    pair.first.n = variables;
    pair.second.chromosome = tmp2;
    pair.second.n = variables;

    return pair;
}

} // genetic_algorithms
 

#endif
