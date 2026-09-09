# AI Novelty and Feasibility Audit

**ExploreBench**, CMPE 249, Assignment 1, Deliverable D
Audit run September 2026 (Claude, Opus 5) against the literature survey in [`LITERATURE_SURVEY.md`](LITERATURE_SURVEY.md). Reproduced below as returned, unedited.

---

## Prompt given to the model

> Evaluate this project for novelty and "red ocean" risk. The project is a controlled comparison of four classical frontier-selection strategies for 2D autonomous exploration in ROS 2 / Gazebo, with ~1,300 randomized episodes, bootstrap CIs and paired significance testing, plus decision-latency and power profiling of each strategy on a Jetson Orin Nano. It proposes no new exploration algorithm. Be critical. Where is this saturated, where is it genuinely open, and what would make it fail?

---

## Verdict

**Novelty: moderate, and located in the methodology and the platform, not in the algorithms.** This project would not be accepted at ICRA as an algorithms paper, and it should not claim to be one. As a benchmarking-and-characterization contribution it occupies a real and defensible gap. **Feasibility: high**, and unusually so for a course project, which is itself part of the argument for it.

---

## 1. Where this is a red ocean

Be blunt about the parts that are saturated.

**Frontier-based exploration as an algorithm space is mature.** The core idea is from 1997 and the cost-utility formulation from 2002 to 2005. Implementing S0 through S3 is engineering, not research. If the project's claim were "we implemented four exploration strategies," it would be a tutorial project and should be rejected on the course's own "no tutorial topics" criterion.

**Learning-based exploration is the actual red ocean here, and you have correctly stayed out of it.** RL and neural-SLAM exploration policies are a crowded, compute-hungry area where a three-person team on borrowed hardware cannot compete with lab groups running large training clusters. Entering it would mean spending the semester on training infrastructure and reporting a policy that underperforms a tuned heuristic. Your exclusion of this branch is the single best scoping decision in the proposal.

**Prediction-based information gain is crowding fast.** MapEx (ICRA 2025) and PIPE (2025) are from the same lab lineage and are actively iterating; a third entrant proposing another map-prediction variant would be entering a race already in progress. You are not doing this, which is correct.

**"Yet another exploration planner" is saturated.** TARE, FUEL, EPIC and their descendants have raised the bar high enough that a new planner from a semester project would be a weaker version of a solved problem.

## 2. Where this is genuinely open

**The evaluation gap is real, and it is documented by the field's own survey.** Brugali et al. (IJRR 2025) systematically establishes that exploration results are not comparable across papers. That is a strong, citable statement of an open problem from a top venue in the same year. A controlled comparison that holds everything but the selection rule fixed is a direct response to it.

**Nobody in the surveyed set does controlled comparison at scale.** All ten recent papers propose a method and evaluate it against baselines they themselves configured. None uses randomized start poses with per-seed pairing, bootstrap confidence intervals, and non-parametric paired testing. The largest sample size found was ten trials per scenario. Twenty seeds per cell with paired Wilcoxon tests and Holm-Bonferroni correction is, oddly, more statistical rigor than the primary literature applies. That is a genuine, if unglamorous, contribution.

**The embedded-latency result is the strongest novelty claim in the proposal, and it should be foregrounded.** Four of the ten surveyed papers treat computation as a first-order design constraint. TARE restructures the representation, PIPE headlines an 83.3% computation reduction, EPIC abandons the volumetric grid for memory reasons, and Lewis et al. (2026) make asymptotic complexity the contribution. Yet all of them profile on workstation-class or NUC-class hardware. **Nobody publishes where a strategy stops meeting a real-time budget on a 15 W SoC.** "Raycast information gain becomes infeasible above X m² of mapped area on an Orin Nano at 15 W" is a number that does not currently exist in the literature, is useful to practitioners, and is squarely inside the course's Deployment track.

**lambda as a swept parameter rather than a fixed constant is under-explored.** Papers pick a weighting and report results at it. Characterizing the parameter's behaviour across environment scales, and specifically testing whether the optimum transfers, is a different and more informative question than "is my cost function better."

## 3. Novelty risks that need managing

**Risk 1: the contribution reads as "we benchmarked existing methods."** This is the framing risk, and it is the most likely reason this project would be judged unambitious. Mitigation: lead with the lambda characterization and the Jetson latency curve, which are *results that do not exist*, and present the strategy comparison as the substrate that makes them measurable. The title should not begin with the word "comparison" in the final report.

**Risk 2: a null result reads as a failed project.** If the strategies are statistically indistinguishable, an unprepared write-up looks like nothing happened. Mitigation: the proposal already commits to a power analysis, which converts a null into a bounded claim ("our design could have detected a 15% difference and did not find one"). Hold to that. Report the power analysis whether or not the result is null.

**Risk 3: someone has already done this and you have not found them.** Search coverage for benchmarking papers is genuinely harder than for method papers, because they are often published in workshops, as technical reports, or as bare GitHub repositories. **Before Week 3, search specifically for: "exploration benchmark ROS 2", "frontier exploration comparison study", the CMU exploration benchmark environments, and any IROS/ICRA workshop on exploration benchmarking from 2023 to 2026.** If a close prior study exists, the project is not dead. It is repositioned as a replication plus the embedded dimension, which is still defensible, but the framing must change and it must change early.

**Risk 4: the Jetson leg is the novelty and it is scheduled last.** Week 12 is late for the highest-value contribution. Mitigation: start bring-up in Week 1 and get a trivial "hello ROS 2 on the Jetson" working long before Week 12, so that Week 12 is profiling and not installation. With a single shared board this matters more, not less, since board time has to be scheduled.

## 4. Feasibility

Unusually strong, and for structural reasons rather than optimism:

- **No training dependency.** Nothing converges or fails to converge. This eliminates the most common failure mode of course projects in this space.
- **Graceful degradation is designed in.** Two of four strategies still gives a comparison; a failed Jetson leg still leaves a full simulation study. The minimum-viable tier is explicitly defined and is a passing project on its own.
- **Hard gates at Weeks 5 and 8** address the two classic failure patterns: components that never integrate, and metrics changed after seeing results.
- **A pre-defined compute fallback** (10 seeds, 3 environments) with the decision date fixed in advance prevents the usual end-of-semester scramble.
- **Every subsystem has two owners.** With a three-person team, single-owner components are the most common cause of a stalled project.

**The real feasibility risks are the boring ones, and the proposal names them:** Nav2 deadlocks, termination never firing on unreachable frontier slivers, and a subtly wrong coverage metric. The third is the dangerous one. A wrong ground-truth free-area extraction silently invalidates every number in the report, and it is not visible in the output. Unit-testing that extraction in Week 5 against hand-measured worlds is correctly listed as a mitigation and should be treated as non-negotiable.

## 5. Recommendation

**Proceed.** The project is well-scoped, honestly positioned, and lands in an underserved corner rather than a crowded one. Three conditions:

1. **Lead with the embedded result.** It is the most novel thing here and the most likely to be publishable as a workshop paper. Frame the study as "what exploration strategies cost on edge hardware," with the controlled comparison as the method rather than the headline.
2. **Run the prior-work search in Risk 3 before Week 3**, while repositioning is still cheap.
3. **Never claim the baselines are state of the art.** State plainly in the introduction that TARE, MapEx, PIPE and EPIC outperform everything implemented here, and that the question is what the *widely deployed default* costs relative to its immediate alternatives. Overclaiming is the fastest way to lose a reviewer's trust, and the honest framing is also the stronger one.

---

## Team response

Accepted in full. Concretely:

- The final report's introduction states that TARE, MapEx, PIPE and EPIC outperform our baselines, before any of our own results are presented.
- The prior-work search in Risk 3 is added to Week 2, ahead of the Week 5 integration gate.
- Jetson bring-up starts Week 1 on the shared board, not Week 12, with board time booked in advance.
- The power analysis is reported regardless of whether the result is null.
