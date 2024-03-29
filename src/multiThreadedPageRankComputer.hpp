#ifndef SRC_MULTITHREADEDPAGERANKCOMPUTER_HPP_
#define SRC_MULTITHREADEDPAGERANKCOMPUTER_HPP_

#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <future>
#include <mutex>
#include <condition_variable>

#include "immutable/network.hpp"
#include "immutable/pageIdAndRank.hpp"
#include "immutable/pageRankComputer.hpp"

namespace {
  class Barrier {
    public:
      Barrier(uint32_t resistance) : m_resistance(resistance) {}

      void reach() {
        std::unique_lock<std::mutex> lk(mutex);

        if (m_resistance > 1) {
          m_resistance--;
          lk.unlock();
          waits();
        } else if (m_resistance == 1) {
          m_resistance--;
          lk.unlock();
          signals();
        }
      }

    private:
      void waits() {
        std::unique_lock<std::mutex> lk(cv_m);
        cv.wait(lk, [this] { return m_resistance == 0; });
      }

      void signals() {
        cv.notify_all();
      }

      volatile uint32_t m_resistance;
      std::condition_variable cv;
      std::mutex cv_m;
      std::mutex mutex;
  };

  void initializePagesIds(Barrier &barrier, Network const &network, std::vector<const Page *> &pages, std::mutex &mut) {
    // Generating names, the work is distributed between threads.
    auto const &generator = network.getGenerator();
    barrier.reach();

    if (pages.size() == 0) {
      return;
    }

    auto page = pages[0]; // auxiliary variable

    while (true) {
      mut.lock();
      if (pages.size() == 0) {
        mut.unlock();
        return;
      }

      page = pages.back();
      pages.pop_back();
      mut.unlock();

      page->generateId(generator);
    }
  }

  void initializeStructures(Network const &network, uint32_t numThreads, std::vector<const Page *> &pages,
                            std::unordered_map<PageId, PageRank, PageIdHash> &pageHashMap,
                            std::unordered_map<PageId, uint32_t, PageIdHash> &numLinks,
                            std::vector<PageId> &danglingNodes,
                            std::unordered_map<PageId, std::vector<PageId>, PageIdHash> &edges) {
    std::vector<std::thread> threadsVector;
    std::mutex mut;

    for (auto const &page : network.getPages()) {
      pages.push_back(&page);
    }

    auto pagesCopy = pages;
    Barrier barrier(numThreads);
    for (uint32_t i = 0; i < numThreads; i++) {
      // Initializing pages` ids.
      threadsVector.push_back(std::thread{initializePagesIds, std::ref(barrier), std::ref(network),
                                          std::ref(pagesCopy), std::ref(mut)});
    }

    for (uint32_t i = 0; i < numThreads; i++) {
      threadsVector[i].join();
    }

    // Initialization of structures needed in MultiThreadedPageRankComputer::computeForNetwork.
    for (auto const &page : network.getPages()) {
      pageHashMap[page.getId()] = 1.0 / network.getSize();
    }

    for (auto const &page : network.getPages()) {
      numLinks[page.getId()] = page.getLinks().size();
    }

    for (auto const &u : numLinks) {
      if (u.second == 0) {
        danglingNodes.push_back(u.first);
      }
    }

    for (auto const &page : network.getPages()) {
      for (auto const &link : page.getLinks()) {
        edges[link].push_back(page.getId());
      }
    }
  }

/* One iteration of an algorithm is evenly distributed between numThreads.
 * Every thread during one iteration is gonna invoke this function.
*/
  void pageRankIteration(Barrier &barrier, std::vector<const Page *> &pages, std::mutex &mut,
                         double alpha, double dangleSum,
                         std::unordered_map<PageId, PageRank, PageIdHash> &pageHashMap,
                         std::unordered_map<PageId, uint32_t, PageIdHash> &numLinks,
                         std::unordered_map<PageId, PageRank, PageIdHash> const &previousPageHashMap,
                         std::unordered_map<PageId, std::vector<PageId>, PageIdHash> &edges,
                         std::promise<double> &differencePromise) {
    double difference = 0;
    auto const networkSize = pages.size();
    barrier.reach();

    if (pages.size() == 0) {
      differencePromise.set_value(difference);
      return;
    }

    auto page = pages[0]; // auxiliary variable

    while (true) {
      mut.lock();
      if (pages.size() == 0) {
        mut.unlock();
        differencePromise.set_value(difference);
        return;
      }

      page = pages.back();
      pages.pop_back();
      mut.unlock();

      PageId pageId = (*page).getId();

      double danglingWeight = 1.0 / networkSize;
      pageHashMap[pageId] = dangleSum * danglingWeight + (1.0 - alpha) / networkSize;

      if (edges.count(pageId) > 0) {
        for (auto const &link : edges[pageId]) {
          pageHashMap[pageId] += alpha * previousPageHashMap.find(link)->second / numLinks[link];
        }
      }
      difference += std::abs(previousPageHashMap.find(pageId)->second - pageHashMap[pageId]);
    }
  }

// Auxiliary function that sums all of the differenceFutures obtained by threads.
  double summaryDifference(uint32_t numThreads, std::future<double> *differenceFutures) {
    double difference = 0;

    for (uint32_t i = 0; i < numThreads; i++) {
      difference += differenceFutures[i].get();
    }

    return difference;
  }

  void singleThreadDangleSum(std::vector<PageId> const &danglingNodes, uint32_t threadNum, uint32_t numThreads,
                             std::promise<double> &dangleSumPromise,
                             std::unordered_map<PageId, PageRank, PageIdHash> const &previousPageHashMap) {
    double dangleSum = 0;

    for (uint32_t i = threadNum; i < danglingNodes.size(); i += numThreads) {
      dangleSum += previousPageHashMap.find(danglingNodes[i])->second;
    }

    dangleSumPromise.set_value(dangleSum);
  }

// Function which obtains dangleSum using numThreads.
  double countDangleSum(std::vector<PageId> const &danglingNodes, uint32_t numThreads,
                        std::unordered_map<PageId, PageRank, PageIdHash> const &previousPageHashMap) {
    double dangleSum = 0;
    std::promise<double> dangleSumPromises[numThreads];
    std::future<double> dangleSumFutures[numThreads];
    std::vector<std::thread> threadsVector;

    for (uint32_t i = 0; i < numThreads; i++) {
      dangleSumFutures[i] = dangleSumPromises[i].get_future();
      threadsVector.push_back(std::thread{singleThreadDangleSum, std::ref(danglingNodes), i, numThreads,
                                          std::ref(dangleSumPromises[i]), std::ref(previousPageHashMap)});
    }

    for (uint32_t i = 0; i < numThreads; i++) {
      dangleSum += dangleSumFutures[i].get();
    }

    for (auto &thread : threadsVector) {
      thread.join();
    }

    return dangleSum;
  }
};

class MultiThreadedPageRankComputer : public PageRankComputer {
  public:
    MultiThreadedPageRankComputer(uint32_t numThreadsArg)
      : numThreads(numThreadsArg) {};

    std::vector<PageIdAndRank>
    computeForNetwork(Network const &network, double alpha, uint32_t iterations,
                      double tolerance) const {
      /* The implementation is basically the same as singleThreadedPageRankComputer::computeForNetwork.
       * The only difference is that the work, is distributed between threads.
      */
      std::vector<std::thread> threadsVector;
      std::unordered_map<PageId, PageRank, PageIdHash> pageHashMap;
      std::unordered_map<PageId, uint32_t, PageIdHash> numLinks;
      std::vector<PageId> danglingNodes;
      std::unordered_map<PageId, std::vector<PageId>, PageIdHash> edges;
      std::vector<const Page *> pages;
      std::mutex mut;

      initializeStructures(network, numThreads, pages, pageHashMap, numLinks, danglingNodes, edges);

      for (uint32_t i = 0; i < iterations; i++) {
        std::unordered_map<PageId, PageRank, PageIdHash> previousPageHashMap = pageHashMap;
        auto pagesCopy = pages;
        Barrier barrier(numThreads);
        threadsVector.clear();
        std::promise<double> differencePromises[numThreads];
        std::future<double> differenceFutures[numThreads];

        double dangleSum = countDangleSum(danglingNodes, numThreads, previousPageHashMap) * alpha;

        for (uint32_t j = 0; j < numThreads; j++) {
          differenceFutures[j] = differencePromises[j].get_future();
          threadsVector.push_back(std::thread{pageRankIteration, std::ref(barrier), std::ref(pagesCopy),
                                              std::ref(mut), alpha, dangleSum,
                                              std::ref(pageHashMap), std::ref(numLinks),
                                              std::ref(previousPageHashMap), std::ref(edges),
                                              std::ref(differencePromises[j])});
        }

        if (summaryDifference(numThreads, differenceFutures) < tolerance) {
          std::vector<PageIdAndRank> result;

          for (auto const &iter : pageHashMap) {
            result.push_back(PageIdAndRank(iter.first, iter.second));
          }

          for (auto &thread : threadsVector) {
            thread.join();
          }

          ASSERT(result.size() == network.getSize(),
                 "Invalid result size=" << result.size() << ", for network" << network);

          return result;
        }

        for (auto &thread : threadsVector) {
          thread.join();
        }
      }

      ASSERT(false, "Not able to find result in iterations=" << iterations);
    }

    std::string getName() const {
      return "MultiThreadedPageRankComputer[" + std::to_string(this->numThreads) + "]";
    }

  private:
    uint32_t numThreads;
};

#endif /* SRC_MULTITHREADEDPAGERANKCOMPUTER_HPP_ */