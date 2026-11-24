# Slides outline (Module 22)

Rule from instructions.md: open with the research question and the Pareto figure; architecture goes at the end or in backup. Every number on a slide comes from `analysis/out/numbers.json` (same keys as `docs/report/report_template.md`). About 12 minutes, 10 slides.

| # | Slide | Content | Source |
|---|-------|---------|--------|
| 1 | Is smarter frontier selection worth its compute on a robot's own computer? | One sentence of context, the question, team | report §1 |
| 2 | The answer in one picture | `fig_q3_pareto.png` (d_95 vs. planner energy above idle, per map), one takeaway line | `analysis/out/` |
| 3 | What we compared | nearest / MRTSP / ARiADNE, one line each; "same SLAM, Nav2, robot, rules" | `docs/common_rules.md` |
| 4 | How | 3 maps (figure strip of `maps/*/gt.png`), 20 fixed start poses, Blocks Q and E, pre-registered | `docs/preregistration.md` |
| 5 | Q1: path to 95% coverage | `fig_q1_coverage.png`, Δd_95 per map with CI | `numbers.q1` |
| 6 | Q2: what it costs on the Jetson | `fig_q2_latency.png`; energy above idle per method | `numbers.q2` |
| 7 | H2: does planning slow down as the map grows? | `fig_h2_scaling.png`, slopes with CIs, deadline line | `numbers.h2` |
| 8 | Q3: when does it pay off? | `fig_q3_break_even.png`, k* vs. a TurtleBot-class k_loco | `numbers.q3` |
| 9 | Limits | the threats-to-validity list, short | report §6 |
| 10 | Takeaways | three bullets, one per question | report §7 |
| B1 | Backup: architecture | harness diagram, Block E layout (Gazebo on the laptop, stack on the Jetson) | report App. A |
| B2 | Backup: protocol details | outcomes, censoring rule, statistics | `docs/preregistration.md` |
| B3 | Backup: demo video | `scripts/make_demo_video.py` output, run IDs | Module 22 step 2 |
