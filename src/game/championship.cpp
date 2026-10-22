#include "championship.h"

#include <algorithm>
#include <random>

namespace bigred {

void Championship::start(const std::vector<TournRound>& rounds,
                         const std::vector<std::string>& playable) {
    rounds_.clear();
    for (const TournRound& r : rounds) {
        TournRound kept = r;
        kept.levels.clear();
        for (const std::string& l : r.levels)
            if (std::find(playable.begin(), playable.end(), l) != playable.end())
                kept.levels.push_back(l);
        if (kept.levels.empty()) continue;
        // Incomplete installations can omit circuits. Keep their filtered
        // round winnable; a complete retail installation retains every track.
        if (kept.need > static_cast<int>(kept.levels.size()))
            kept.need = static_cast<int>(kept.levels.size());
        if (kept.need < 1) kept.need = 1;
        rounds_.push_back(std::move(kept));
    }
    done_.clear();
    round_ = wins_ = raced_ = 0;
    over_ = won_ = false;
}

void Championship::start_random(const std::vector<TournRound>& rounds,
                                const std::vector<std::pair<std::string, int>>& playable_difficulty,
                                uint32_t seed) {
    std::vector<std::string> playable;
    for (const auto& p : playable_difficulty) playable.push_back(p.first);
    start(rounds, playable);   // the shipped rounds, filtered: their sizes and needs are kept
    auto diff = [&](const std::string& l) {
        for (const auto& p : playable_difficulty)
            if (p.first == l) return p.second;
        return -1;
    };
    // The circuits in use, by difficulty; an unknown one keeps its shipped place.
    std::vector<std::string> pool[3];
    for (const TournRound& r : rounds_)
        for (const std::string& l : r.levels) {
            const int d = diff(l);
            if (d >= 0 && d <= 2) pool[d].push_back(l);
        }
    std::mt19937 rng(seed);
    for (std::vector<std::string>& p : pool) std::shuffle(p.begin(), p.end(), rng);
    std::vector<std::string> order;
    for (const std::vector<std::string>& p : pool) order.insert(order.end(), p.begin(), p.end());
    size_t next = 0;
    for (TournRound& r : rounds_)
        for (std::string& l : r.levels)
            if (diff(l) >= 0 && next < order.size()) l = order[next++];
    // Within a round, the file's order is what the list shows: keep it tidy, easiest first.
    for (TournRound& r : rounds_)
        std::stable_sort(r.levels.begin(), r.levels.end(),
                         [&](const std::string& a, const std::string& b) { return diff(a) < diff(b); });
}

int Championship::need() const {
    return valid() ? rounds_[static_cast<size_t>(round_)].need : 0;
}

int Championship::round_size() const {
    return valid() ? static_cast<int>(rounds_[static_cast<size_t>(round_)].levels.size()) : 0;
}

std::vector<std::string> Championship::available() const {
    std::vector<std::string> out;
    if (!valid() || over_) return out;
    for (const std::string& l : rounds_[static_cast<size_t>(round_)].levels)
        if (std::find(done_.begin(), done_.end(), l) == done_.end()) out.push_back(l);
    return out;
}

Championship::Outcome Championship::record(const std::string& level, int place) {
    if (!valid() || over_) return Outcome::Continue;
    // Raced, and marked - won or not.
    ++raced_;
    if (std::find(done_.begin(), done_.end(), level) == done_.end()) done_.push_back(level);
    if (place == 1) ++wins_;
    // The original's checks, in their order: the title first, then the elimination, then
    // the next round.
    if (wins_ >= need()) {
        if (final_round()) {
            over_ = won_ = true;
            return Outcome::Champion;
        }
        ++round_;
        wins_ = raced_ = 0;
        return Outcome::NextRound;
    }
    if ((round_size() - raced_) + wins_ < need()) {
        over_ = true;
        return Outcome::Eliminated;
    }
    return Outcome::Continue;
}

} // namespace bigred
