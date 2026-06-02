---
layout: gtsam-post
title:  "Certifiable Optimization, Part 3: Global Solvers for 3D Vision"
---

Author: [Frank Dellaert](https://dellaert.github.io/)

<!-- - TOC -->
{:toc}

This third and final ICRA-week post zooms out to the broader global-solvers ecosystem.
[Part 1]({% post_url 2026-06-01-icra-for-workshop %}) focused on our chordal-sparsity work, which uses Bayes-tree structure to make SDP relaxations more scalable.
Part 2 focused on David Rosen's group's [Certifiable Factor Graph Optimization](https://openreview.net/forum?id=hAtI0KkdAg), which approaches related factor-graph problems through Burer-Monteiro-style low-rank optimization.
But these are only two efforts in a broader push toward global solvers for 3D vision and robot perception, a lot of it started by Luca Carlone over the last 10 years. Two events of note:

- On Wednesday, June 3, 2026, at 11:00 in Hall A1, Luca Carlone will give the ICRA keynote [Maps, Memory, and Tasks: Toward Spatial AI for the Next Generation of Robots](https://2026.ieee-icra.org/event/keynote-3/).
- On Monday of this week, Heng Yang gave the workshop keynote [Scaling Semidefinite Relaxations for Robot Perception and Control](https://sites.google.com/robotics.utias.utoronto.ca/icra26-frontiers-optimization/schedule) at the [Frontiers of Optimization for Robotics workshop](https://sites.google.com/robotics.utias.utoronto.ca/icra26-frontiers-optimization/).

## Global solvers for 3D vision

The workshop paper [Global Solvers for 3D Vision: Foundations, Frontiers, and a Call to the Robotics Community](https://openreview.net/forum?id=D1bVYYUh8m) is a useful entry point because it argues that global solvers are mature but still underused in robotics.
Zhenjun Zhao and Javier Civera frame branch-and-bound, convex relaxation, and graduated non-convexity around geometric estimation problems that show up constantly in robot perception.
For a longer map, see the arXiv survey [Advances in Global Solvers for 3D Vision](https://arxiv.org/abs/2602.14662), where Heng is an author together with Zhao, Bangyan Liao, Yingping Zeng, Shaocheng Yan, Yingdong Gu, Peidong Liu, Yi Zhou, Haoang Li, and Civera.

<figure class="center" style="width: 100%; max-width: 1100px; text-align: center;">
  <img src="/assets/images/global-solvers-2026/global-solvers-taxonomy.png"
    alt="Taxonomy of global solvers for 3D vision, including branch-and-bound, convex relaxation, graduated non-convexity, comparative analysis, and applications."
    style="width: 100%; display: block; margin-left: auto; margin-right: auto;" />
  <figcaption>Global solvers for 3D vision span branch-and-bound, convex relaxation, and graduated non-convexity, with applications from Wahba's problem to bundle adjustment. Figure adapted from the Zhao et al. survey.</figcaption>
</figure>
<br />

Global solvers are not a single trick for one problem.
They show up in Wahba's problem, vanishing-point estimation, absolute and relative pose, 3D registration, rotation and translation averaging, triangulation, pose-graph optimization, and bundle adjustment.
For GTSAM users, the convex-relaxation branch is especially natural, because many estimation problems can be expressed as QCQPs and then relaxed into semidefinite programs.
The practical challenge is to preserve the sparsity and geometry that factor graphs already expose, so that a certificate is not bought at the price of an unusable solver.

## A short lineage

One way to see the field's trajectory is through rotation estimation, pose-graph optimization, and registration.
Our 2015 ICRA paper, [Initialization Techniques for 3D SLAM](https://repository.gatech.edu/entities/publication/ae22ec18-65d1-4075-8c0b-55c694ac5467), was about making hard nonconvex SLAM problems behave better by separating out the rotation-estimation structure before running local pose-graph optimization.
At MIT, [SE-Sync](https://arxiv.org/abs/1612.07386) showed how pose synchronization over `SE(d)` could be solved efficiently while certifying global optimality in the relevant noise regime.
[Shonan Rotation Averaging](https://arxiv.org/abs/2008.02737) then connected SDP relaxation and manifold optimization to large-scale rotation averaging.
[TEASER](https://arxiv.org/abs/2001.07715), by Heng Yang, Jingnan Shi, and Luca Carlone, pushed certifiable registration toward extreme outlier robustness.
These are examples rather than a canon, but they help explain why the current workshop papers feel timely.

## Back to GTSAM

This series ends where GTSAM has to pick up: with tooling.
Part 1 emphasized chordal SDP structure, Part 2 emphasized low-rank factor-graph certification, and Part 3 points to the larger global-solvers ecosystem.
The recent [QP and QCQP support in GTSAM]({% post_url 2026-05-13-qp-qcqp-in-gtsam %}) is a preview of the modeling layer needed to lift, relax, decompose, and eventually certify factor-graph problems.
We hope to bring certifiable optimization to GTSAM soon, and I hope more robotics people join the field.

## Further browsing

- [Post: Certifiable Optimization at the ICRA Workshop]({% post_url 2026-06-01-icra-for-workshop %})
- [OpenReview: Certifiable Factor Graph Optimization](https://openreview.net/forum?id=hAtI0KkdAg)
- [Workshop paper: Global Solvers for 3D Vision](https://openreview.net/forum?id=D1bVYYUh8m)
- [ArXiv: Advances in Global Solvers for 3D Vision](https://arxiv.org/abs/2602.14662)
- [Frontiers of Optimization for Robotics workshop schedule](https://sites.google.com/robotics.utias.utoronto.ca/icra26-frontiers-optimization/schedule)
- [ICRA 2026 keynote: Luca Carlone, Maps, Memory, and Tasks](https://2026.ieee-icra.org/event/keynote-3/)
- [ArXiv: SE-Sync](https://arxiv.org/abs/1612.07386)
- [ArXiv: TEASER](https://arxiv.org/abs/2001.07715)
- [ArXiv: Shonan Rotation Averaging](https://arxiv.org/abs/2008.02737)
- [Post: Quadratic Programs and QCQPs in GTSAM]({% post_url 2026-05-13-qp-qcqp-in-gtsam %})

_Disclosure: AI was used to help draft this post and prepare the figure._
