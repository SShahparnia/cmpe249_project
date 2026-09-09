# Literature & SOTA Survey

**ExploreBench** · CMPE 249 · Assignment 1, Deliverable B
Ten papers published 2023–2026, surveyed September 2026 across arXiv, IEEE Xplore, and journal sites.

| # | Work | Year | Venue | Category | Code |
|---|---|---|---|---|---|
| 1 | Brugali et al. — Exploration strategies & requirements: systematic mapping study | 2025 | IJRR | Survey | — |
| 2 | Placed et al. — Survey on active SLAM | 2023 | IEEE T-RO | Survey | — |
| 3 | Cao et al. — Representation granularity enables time-efficient exploration (TARE) | 2023 | Science Robotics | SOTA planner | ✅ |
| 4 | Ho et al. — MapEx: probabilistic information gain from map predictions | 2025 | ICRA | Prediction-based IG | ✅ |
| 5 | Baek et al. — PIPE Planner: pathwise information gain | 2025 | arXiv 2503.07504 | Prediction-based IG | — |
| 6 | Liu et al. — Real-time map optimization and improved frontier costs | 2025 | Scientific Reports | Classical frontier cost | — |
| 7 | Calzolari et al. — Graph-based RL with frontier-potential reward | 2025 | arXiv 2504.11907 | Learning-based | — |
| 8 | Zhan et al. — Semantic exploration with panoramic LiDAR-camera fusion | 2025 | IEEE RA-L | Semantic / system | — |
| 9 | Geng et al. — EPIC: lightweight LiDAR-based exploration at scale | 2025 | IEEE RA-L | Efficiency / scaling | ✅ |
| 10 | Lewis et al. — Asymptotically-bounded 3D frontier exploration | 2026 | arXiv 2604.03008 | Complexity / IG | — |

---

**1. Brugali, D., Muratore, L., & De Luca, A. (2025).** *Mobile robots exploration strategies and requirements: a systematic mapping study.* IJRR. [DOI](https://doi.org/10.1177/02783649241313471)

A systematic mapping of the exploration literature, classifying strategies by the requirements they were designed against. It documents that results are reported on incompatible simulators, robots, and metrics, making cross-paper comparison largely impossible. This is the strongest citation for our motivation: rather than adding another incomparable data point, we hold everything but the selection rule fixed.

**2. Placed, J. A., et al. (2023).** *A survey on active SLAM: state of the art and new frontiers.* IEEE T-RO 39(3), 1686–1705. [arXiv](https://arxiv.org/abs/2207.00254)

The definitive modern survey of active SLAM, organizing the field into frontier-based, information-theoretic, and learning-based families and formalizing the coupling between where a robot goes and how well it localizes. We adopt its taxonomy for related work, and it supplies the theoretical basis for RQ3: a strategy optimized purely for coverage speed implicitly trades away localization quality, which few papers measure.

**3. Cao, C., et al. (2023).** *Representation granularity enables time-efficient autonomous exploration in large, complex worlds.* Science Robotics 8(80), eadf0970. [DOI](https://doi.org/10.1126/scirobotics.adf0970) · [code](https://github.com/caochao39/tare_planner)

The journal extension of TARE: a fine-grained local representation inside a sliding window plus a coarse global one outside it, arguing that this granularity split — not the planner — is what makes exploration scale in real time. TARE substantially outperforms every strategy we implement, and our report says so plainly. It also frames RQ4: if latency blows up, the fix is to change representation granularity, not micro-optimize the raycaster. We do not reimplement it; that is not a one-semester task.

**4. Ho, C., et al. (2025).** *MapEx: indoor structure exploration with probabilistic information gain from global map predictions.* ICRA 2025. [arXiv](https://arxiv.org/abs/2409.15590) · [code](https://github.com/castacks/MapEx)

Predicts the unobserved part of the map from the observed part, generates an ensemble of predictions, and uses their variance to build a probabilistic sensor model for information gain. Reports beating nearest-frontier by 25.4% and a representative prediction-based method by 12.4% on the KTH dataset. That number gives our S1 baseline a scale to be read against, and it identifies the exact weakness of our S2 estimator: counting unknown cells treats all unknown space as equally informative, which in a building it is not. We exclude map prediction itself because it requires a trained model.

**5. Baek, S., Moon, B., Kim, S., Cao, M., Ho, C., Scherer, S., & Jeon, J. H. (2025).** *PIPE Planner: pathwise information gain with map predictions for indoor robot exploration.* [arXiv:2503.07504](https://arxiv.org/abs/2503.07504)

Integrates sensor coverage along the *whole planned path* rather than at the destination viewpoint, and reports an 83.3% computation-time reduction on large maps via polygon merging and flood-fill. It names the modelling error at the heart of our S2 and S3: scoring only the terminal viewpoint ignores everything seen while driving there, undervaluing distant frontiers reached through unexplored corridors. Our strategies are pointwise by design, and this is how we state that limitation honestly. Its baseline set (Nearest, NBV-2D, PW-NBV-2D, UPEN, MapEx) maps closely enough onto our S1/S2 to sanity-check our results.

**6. Liu, C., et al. (2025).** *Enhancing autonomous exploration for robotics via real-time map optimization and improved frontier costs.* Scientific Reports 15, 12261. [link](https://www.nature.com/articles/s41598-025-97231-9)

Applies bilateral filtering and dilation to remove spurious frontiers, then replaces single-distance frontier cost with a function combining path length, sensor range, and information gain. Evaluated in corridor, maze, and cluttered environments plus a real wheeled robot, ten trials per scenario. This is the closest recent analogue to our setup: it confirms multi-term frontier costs are still active research in 2025, and its environment choices independently validate our E3–E5 design. It also shows the gap we fill — it tests one cost function over ten trials without a sweep of its weighting or a variance treatment, where we sweep λ and report bootstrap CIs.

**7. Calzolari, G., Sumathy, V., Kanellakis, C., & Nikolakopoulos, G. (2025).** *A graph-based reinforcement learning approach with frontier-potential-based reward for safe cluttered environment exploration.* [arXiv:2504.11907](https://arxiv.org/abs/2504.11907)

Learns a greedy exploration policy with attention-based GNN layers over a graph of robot, goal, and frontier nodes, wrapped in a safety shield. Reaches ~95% coverage in 2,000 steps across 1,000 simulated environments. Our representative of the learning-based family, and the reason we can justify excluding it briefly: the evaluation shows policies *can* be learned, not that they beat a tuned cost–utility heuristic under matched conditions. A rigorous classical baseline — what we produce — is a prerequisite for that comparison being meaningful.

**8. Zhan, X., et al. (2025).** *Semantic exploration and dense mapping of complex environments using ground robot with panoramic LiDAR-camera fusion.* IEEE RA-L (accepted Aug 2025). [arXiv](https://arxiv.org/abs/2505.22880)

Pursues geometric coverage and multi-view semantic object observation simultaneously, with a priority-driven decoupled local sampler and a "safe aggressive exploration state machine" for recovering from false collision detections. Validated in Isaac Sim and on a Boston Dynamics Spot. Two takeaways for us: once a second objective enters, the single-scalar-utility formulation all four of our strategies share becomes a modelling choice rather than the obvious framing; and its recovery state machine is a direct warning about our highest-likelihood risk — exploration systems fail on stuck-robot states far more often than on bad frontier choices.

**9. Geng, S., Ning, Z., Zhang, F., & Zhou, B. (2025).** *EPIC: a lightweight LiDAR-based AAV exploration framework for large-scale scenarios.* IEEE RA-L. [arXiv](https://arxiv.org/abs/2410.14203) · [code](https://github.com/Robotics-STAR-Lab/EPIC)

Abandons the volumetric grid entirely, building an observation map from point-cloud quality and an incremental topological graph over points, with substantially reduced memory and computation. The strongest recent evidence for our RQ4 premise that the representation, not the selection rule, consumes the compute budget. It anticipates the wall we expect to hit: a 5 cm grid over a large world is ~4 M cells inside the Orin Nano's ~6 GB usable unified memory. Our result is the measurement that motivates such a redesign; theirs is one such redesign.

**10. Lewis, J., Basiri, M., & Lima, P. U. (2026).** *Asymptotically-bounded 3D frontier exploration enhanced with Bayesian information gain.* [arXiv:2604.03008](https://arxiv.org/abs/2604.03008)

Folds frontier detection into the forward sensor model so its cost is `O(|F|)` in the number of frontiers rather than scaling with map update volume, then replaces geometric cell-counting with a Gaussian-process information-gain estimate. Compared against a multi-resolution frontier planner, a hybrid OctoMap approach, and EPIC. The most direct and most recent methodological precedent for RQ4 — it treats frontier-detection complexity as the headline contribution. It is also a caution on our own claims: its simulations ran on a 32 GB HPC node and its flights on an Intel NUC i7, not a 15 W SoC, so its complexity results do not say where a strategy breaks on an Orin Nano. That gap is what we measure.

---

## What the survey establishes

**Our baselines are baselines, not SOTA.** TARE (#3), MapEx (#4), PIPE (#5), and EPIC (#9) all outperform greedy frontier selection by published margins. We are not claiming nearest-frontier is state of the art; we ask what the *default* costs relative to its immediate alternatives, on matched conditions, with variance reported.

**Classical frontier-cost design is still live.** #6 published a multi-term frontier cost in 2025 and #10 a frontier-detection complexity result in 2026, so our S3 formulation is a current object of study.

**No one in this set runs a controlled multi-strategy comparison.** All ten propose a method and evaluate against baselines. None uses randomized starts with per-seed pairing, bootstrap confidence intervals, and non-parametric paired testing; the largest sample size found was ten trials per scenario (#6).

**Compute is the recurring constraint, unmeasured on our hardware class.** #3 restructures the representation for it, #5 headlines an 83.3% computation reduction, #9 drops the grid for memory, #10 makes asymptotic complexity the contribution. None reports where a strategy stops meeting a real-time budget on a 15 W embedded SoC. That is RQ4, and it is the clearest open slot the survey revealed.
