# Literature and SOTA Survey

**Project:** ExploreBench
**Course:** CMPE 249, Assignment 1, Deliverable B
**Team:** Shervan Shahparnia, Joey Manivong, Aaron Sam

Ten papers published between 2023 and 2026. Searched September 2026 across arXiv, IEEE Xplore, and journal sites.

## Overview

| # | Work | Year | Venue | Category | Code |
|---|---|---|---|---|---|
| 1 | Brugali et al., Exploration strategies and requirements | 2025 | IJRR | Survey | no |
| 2 | Placed et al., Survey on active SLAM | 2023 | IEEE T-RO | Survey | no |
| 3 | Cao et al., Representation granularity (TARE) | 2023 | Science Robotics | SOTA planner | yes |
| 4 | Ho et al., MapEx | 2025 | ICRA | Prediction-based gain | yes |
| 5 | Baek et al., PIPE Planner | 2025 | arXiv 2503.07504 | Prediction-based gain | no |
| 6 | Liu et al., Improved frontier costs | 2025 | Scientific Reports | Classical frontier cost | no |
| 7 | Calzolari et al., Graph-based RL exploration | 2025 | arXiv 2504.11907 | Learning-based | no |
| 8 | Zhan et al., Semantic exploration and dense mapping | 2025 | IEEE RA-L | Semantic system | no |
| 9 | Geng et al., EPIC | 2025 | IEEE RA-L | Efficiency and scaling | yes |
| 10 | Lewis et al., Asymptotically-bounded 3D frontier exploration | 2026 | arXiv 2604.03008 | Complexity and gain | no |

## 1. Mobile robots exploration strategies and requirements: a systematic mapping study

Brugali, D., Muratore, L., and De Luca, A. (2025). *International Journal of Robotics Research*.
https://doi.org/10.1177/02783649241313471

**What it does.** This paper surveys the exploration literature in a systematic way. It sorts strategies by the requirements they were built to satisfy, and it records how each one was tested.

**Why it matters to us.** It shows, with a repeatable method instead of an opinion, that exploration papers use incompatible simulators, robots, and metrics. That makes it nearly impossible to compare results across papers. This is the strongest support for our motivation. Instead of adding one more result that nobody can compare, we hold every part of the system fixed except the selection rule.

## 2. A survey on active simultaneous localization and mapping: state of the art and new frontiers

Placed, J. A., Strader, J., Carrillo, H., Atanasov, N., Indelman, V., Carlone, L., and Castellanos, J. A. (2023). *IEEE Transactions on Robotics*, 39(3), 1686 to 1705.
https://arxiv.org/abs/2207.00254

**What it does.** This is the standard modern survey of active SLAM. It splits the field into three families: frontier-based, information-theoretic, and learning-based. It also describes formally how a robot's movement choices affect how well it can localize itself.

**Why it matters to us.** We use its three-family taxonomy to organize our related-work section, and it tells us exactly which part of the design space our four strategies cover. It also gives us the theory behind RQ3. If a robot picks routes only to maximize coverage speed, it revisits fewer places, so the SLAM system gets fewer loop closures and the pose estimate gets worse. Very few papers actually measure that cost, and we do.

## 3. Representation granularity enables time-efficient autonomous exploration in large, complex worlds

Cao, C., Zhu, H., Yang, F., Xia, Y., Choset, H., Oh, J., and Zhang, J. (2023). *Science Robotics*, 8(80), eadf0970.
https://doi.org/10.1126/scirobotics.adf0970 · Code: https://github.com/caochao39/tare_planner

**What it does.** This is the journal version of TARE. The planner keeps a detailed map inside a small window around the robot and a rough map everywhere else. The authors argue that this split in detail level, not the planner itself, is what lets exploration stay fast in large spaces.

**Why it matters to us.** TARE beats every strategy we implement, and our report will say so directly instead of hiding it. The paper also gives us the right way to think about RQ4. If our decision latency grows too large, the fix suggested here is to reduce the detail level of the map representation, not to squeeze more speed out of the raycasting code. We do not reimplement TARE. Rebuilding a Science Robotics system is not realistic for three students in one semester, and a poor reimplementation would produce a misleading comparison.

## 4. MapEx: indoor structure exploration with probabilistic information gain from global map predictions

Ho, C., Kim, S., Moon, B., Parandekar, A., Harutyunyan, N., Wang, C., Sycara, K., Best, G., and Scherer, S. (2025). *ICRA 2025*.
https://arxiv.org/abs/2409.15590 · Code: https://github.com/castacks/MapEx

**What it does.** MapEx predicts what the unseen part of the building probably looks like, based on the part already mapped. It generates several such predictions, then uses the disagreement between them to build a probabilistic sensor model for information gain. So gain is estimated over a plausible finished map rather than over blank unknown space.

**Why it matters to us.** MapEx reports beating nearest-frontier by 25.4 percent and beating another prediction-based method by 12.4 percent on the KTH floorplan dataset. That first number gives our S1 baseline a scale to be read against. The paper also names the exact weakness in our S2 estimator: counting unknown cells treats every unknown cell as equally useful, and inside a building that is clearly false. Some unknown cells sit behind a wall in a closet, and some open into a whole unexplored wing. We do not use map prediction ourselves, because it needs a model trained on floorplan data, and training dependencies are the risk our scope rules out.

## 5. PIPE Planner: pathwise information gain with map predictions for indoor robot exploration

Baek, S., Moon, B., Kim, S., Cao, M., Ho, C., Scherer, S., and Jeon, J. H. (2025). arXiv 2503.07504.
https://arxiv.org/abs/2503.07504

**What it does.** PIPE adds up the sensor coverage the robot gains along its whole planned path, not just at the final viewpoint. It combines this with map prediction so that gain in unknown space is not overestimated. The authors also report cutting computation time by 83.3 percent on large maps using polygon merging and flood-fill.

**Why it matters to us.** This paper names a modeling error that sits at the center of our S2 and S3 strategies. We score only the destination, which ignores everything the robot observes while driving there. That systematically undervalues a distant frontier reached by driving down an unexplored hallway. Our strategies are pointwise on purpose, and this is the citation we use to state that limitation honestly. The paper's baseline set is also close to ours. Its Nearest and NBV-2D baselines line up with our S1 and S2, so we can check that our numbers land in a believable range.

## 6. Enhancing autonomous exploration for robotics via real-time map optimization and improved frontier costs

Liu, C., Zhang, D., Liu, W., Sui, X., Huang, Y., Ma, X., Yang, X., and Wang, X. (2025). *Scientific Reports*, 15, 12261.
https://www.nature.com/articles/s41598-025-97231-9

**What it does.** The authors clean up the occupancy grid in real time using bilateral filtering and dilation, which removes junk frontiers while keeping real edges intact. They then replace the usual single distance term in the frontier cost with a function that combines path length, sensor range, and information gain. They test in corridor, maze, and cluttered simulated environments plus a real wheeled robot, with ten trials per scenario.

**Why it matters to us.** This is the closest recent match to our own setup, and it helps us in two directions. It confirms that multi-term frontier cost functions are still an active research topic in 2025, so our S3 formulation is current rather than historical. Its choice of corridor, maze, and cluttered environments also independently supports our E3 to E5 designs. At the same time it shows the gap we fill. It tests one proposed cost function over ten trials, with no sweep of the weighting between its terms and no reported variance treatment. That paper answers "is my cost function better." We ask a different question: "what does the weighting parameter actually do, and does the best value transfer to a new environment."

## 7. A graph-based reinforcement learning approach with frontier potential based reward for safe cluttered environment exploration

Calzolari, G., Sumathy, V., Kanellakis, C., and Nikolakopoulos, G. (2025). arXiv 2504.11907.
https://arxiv.org/abs/2504.11907

**What it does.** The robot, its goal, and the candidate frontiers are represented as a graph. An attention-based graph neural network learns a greedy exploration policy over that graph, and a safety shield overrides any action it judges unsafe. The system reaches about 95 percent coverage in 2,000 steps across 1,000 simulated environments, with the shield stepping in on under 15 percent of actions.

**Why it matters to us.** This is our example of the learning-based family, and it lets us justify excluding that family in a sentence rather than a page. Look closely at what the evaluation shows. It reports coverage curves in a procedurally generated outdoor domain without a head-to-head comparison against tuned classical planners. That is not a flaw in the paper. It describes where this branch of the field currently stands: it is proving that policies can be learned, not that they beat a well-tuned cost-utility heuristic under matched conditions. Someone has to produce a rigorous classical baseline before that comparison can mean anything, and that is what we produce.

## 8. Semantic exploration and dense mapping of complex environments using ground robot with panoramic LiDAR-camera fusion

Zhan, X., Zhou, S., Yang, Q., Zhao, Y., Liu, H., Ramineni, S. C., and Shimada, K. (2025). *IEEE Robotics and Automation Letters*, accepted August 2025.
https://arxiv.org/abs/2505.22880

**What it does.** This system chases two goals at the same time: cover the space geometrically, and observe labeled objects from several angles. A priority-driven decoupled local sampler keeps the two goals from interfering, and a safe aggressive exploration state machine lets the robot push into unmapped areas and recover when it detects a collision that is not real. Tested in Isaac Sim and on a Boston Dynamics Spot with an Ouster OS-1-128 LiDAR.

**Why it matters to us.** Two lessons. First, once a second objective enters the problem, the single scalar utility score that all four of our strategies share stops being the obvious framing and becomes a modeling choice we should defend. Our lambda then describes one axis of a larger trade-off surface, not the whole thing. Second, the recovery state machine is a direct warning about our most likely failure. Exploration systems break far more often on a stuck robot or a false collision than on a bad frontier choice, so those states need real engineering and need to be logged as data.

## 9. EPIC: a lightweight LiDAR-based AAV exploration framework for large-scale scenarios

Geng, S., Ning, Z., Zhang, F., and Zhou, B. (2025). *IEEE Robotics and Automation Letters*.
https://arxiv.org/abs/2410.14203 · Code: https://github.com/Robotics-STAR-Lab/EPIC

**What it does.** EPIC drops the volumetric grid completely. It builds an observation map straight from the quality of the incoming point cloud and grows an incremental topological graph over those points. This cuts memory use and computation enough to plan in real time across large environments.

**Why it matters to us.** This is the strongest recent evidence for the premise behind RQ4, which is that the map representation, not the selection rule, is what eats the compute budget. It also predicts the wall we expect to hit. Our occupancy grid at 5 cm resolution over a large world is roughly 4 million cells, and the Orin Nano has about 6 GB of usable unified memory once JetPack and the ROS stack are loaded. EPIC's answer to that class of problem is to abandon the dense grid entirely. Our contribution is the measurement that would justify such a redesign. Theirs is one example of the redesign itself.

## 10. Asymptotically-bounded 3D frontier exploration enhanced with Bayesian information gain

Lewis, J., Basiri, M., and Lima, P. U. (2026). arXiv 2604.03008.
https://arxiv.org/abs/2604.03008

**What it does.** The authors move frontier detection inside the forward sensor model. This makes its cost scale as O(|F|) in the number of frontiers rather than growing with how much of the map was updated. They then swap rigid geometric cell counting for a Gaussian process regressor that estimates information gain probabilistically. They compare against a multi-resolution frontier planner, a hybrid OctoMap method, and EPIC.

**Why it matters to us.** This is the closest and most recent methodological precedent for RQ4. It treats the computational complexity of frontier detection as the headline contribution, which is exactly the quantity we plan to measure on embedded hardware. It also independently supports our criticism of S2, that counting unknown cells is a crude way to estimate gain. Finally, it is a useful caution about our own claims. Its simulations ran on an HPC node with 32 GB of RAM and its flight tests on an Intel NUC with a Core i7. Complexity results measured on that class of hardware do not tell you where a strategy breaks on a 15 W SoC. That is the gap our measurement fills.

## What the survey establishes

**Our baselines are baselines, not state of the art.** TARE, MapEx, PIPE, and EPIC all beat greedy frontier selection by published margins. We are not claiming nearest-frontier is state of the art. We are asking what the widely deployed default costs compared to its immediate alternatives, under matched conditions, with variance reported.

**Classical frontier cost design is still an active topic.** Paper 6 published a multi-term frontier cost in 2025, and paper 10 published a frontier detection complexity result in 2026. Our S3 formulation is something people are still studying, not a historical artifact.

**Nobody in this set runs a controlled multi-strategy comparison.** All ten papers propose a method and test it against baselines they configured themselves. None uses randomized start poses with per-seed pairing, bootstrap confidence intervals, and non-parametric paired testing. The largest sample size we found was ten trials per scenario, in paper 6.

**Compute is the recurring constraint, and nobody measures it on our hardware class.** Paper 3 restructures the representation to control it. Paper 5 makes an 83.3 percent computation reduction a headline result. Paper 9 abandons the grid for memory reasons. Paper 10 makes asymptotic complexity the contribution. Every one of them establishes that this matters, and none reports where a strategy stops meeting a real-time budget on a 15 W embedded SoC. That measurement is RQ4, and it is the clearest open slot the survey revealed.
