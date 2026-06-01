---
layout: gtsam-post
title:  "Certifiable Factor Graph Optimization"
---

Author: [Jason Xu](https://zhexin1904.github.io/) and [David M. Rosen](https://david-m-rosen.github.io/)

<!-- - TOC -->
{:toc}

Certifiable optimization is becoming a practical robotics topic because we can now ask for both efficient estimates and evidence that those estimates are globally optimal. Frank's [companion post on chordal sparsity]({% post_url 2026-06-01-icra-for-workshop %}) highlights one way to make SDP relaxations exploit Bayes-tree structure; here we describe complementary work from Northeastern that makes certifiable estimation look and feel like ordinary factor graph optimization.

Our workshop paper is:

> [Certifiable Factor Graph Optimization](https://arxiv.org/abs/2603.01267), by Jason (Zhexin) Xu, Nikolas R. Sanderson, Hanna Jiamei Zhang, and David M. Rosen. Workshop page: [OpenReview](https://openreview.net/forum?id=hAtI0KkdAg).

## Factor graphs with certificates

Factor graphs are the modeling language roboticists already use for state estimation, but standard factor graph inference usually relies on local optimization. This is the right tradeoff for many systems: local solvers are fast, sparse, and easy to deploy in libraries such as GTSAM, g2o, and Ceres. The drawback is reliability. A local solver can return a bad local minimum, and the user may not know that anything went wrong.

Certifiable estimators provide the missing reliability layer, but they have historically required too much custom optimization work. The usual pipeline builds a convex relaxation, applies a low-rank Burer-Monteiro factorization, designs a problem-specific Riemannian optimizer, and wraps the result in the Riemannian Staircase. That machinery can recover globally optimal solutions and prove it after the fact, but implementing it for each new estimation problem has often required months of specialized work.

Our main observation is that Shor relaxations and Burer-Monteiro factorizations preserve factor graph structure. If the original estimation problem can be written as a QCQP with a factor graph model, then the lifted problem used by the Riemannian Staircase has the same variable-factor connectivity. The lifted variables and lifted factors are algebraic counterparts of the original variables and factors, so the local optimization at each staircase level can itself be implemented as a factor graph optimization.

<figure class="center" style="width: 100%; max-width: 1100px; text-align: center;">
  <img src="/assets/images/certifiable-factor-graphs/framework.png"
    alt="Framework diagram showing an input factor graph and QCQP lifted into a Burer-Monteiro factor graph inside a Riemannian Staircase."
    style="width: 100%; display: block; margin-left: auto; margin-right: auto;" />
  <figcaption>Certifiable factor graph optimization keeps the same factor graph connectivity after lifting the QCQP and embedding it in the Riemannian Staircase.</figcaption>
</figure>
<br />

## Lifted building blocks

The practical consequence is that a certifiable estimator can be assembled by replacing ordinary factor graph components with lifted versions. In the paper we give explicit lifted constructions for common robotics variables, including rotations, translations, and unit vectors, and for common factors such as relative rotation, relative translation, and point-to-point range measurements. Once those lifted pieces exist, the same factor graph workflow can instantiate the local optimization problems required by the Riemannian Staircase.

This reframes the Riemannian Staircase as an outer loop around familiar factor graph optimization. At each rank, the algorithm solves a lifted factor graph problem, tests whether the recovered point certifies optimality of the corresponding SDP relaxation, and only increases the rank when the certificate fails. When the initial rank is enough, the method behaves much like a standard local factor graph solve plus a certificate check. When it is not enough, the Staircase adds the extra degrees of freedom needed to escape the local solution.

## Experiments

The experiments test whether the factor-graph formulation reproduces specialized certifiable estimators without giving up the efficiency of local solvers. We evaluated pose graph optimization, landmark SLAM, and range-aided SLAM benchmarks, comparing against SE-Sync, Landmark SE-Sync, and CORA, as well as GTSAM's Levenberg-Marquardt optimizer.

<figure class="center" style="width: 100%; max-width: 980px; text-align: center;">
  <img src="/assets/images/certifiable-factor-graphs/benchmarks.png"
    alt="Globally optimal solutions for pose graph optimization, landmark SLAM, and range-aided SLAM benchmarks."
    style="width: 100%; display: block; margin-left: auto; margin-right: auto;" />
  <figcaption>Globally optimal or certifiably near-optimal solutions recovered on pose graph optimization, landmark SLAM, and range-aided SLAM benchmarks.</figcaption>
</figure>
<br />

The certifiable factor graph implementation matches the specialized certifiable solvers while using a much more general construction. On pose graph optimization and landmark SLAM, the tested SDP relaxations were exact and the method recovered certifiably globally optimal solutions. On range-aided SLAM, where the relaxation is typically not exact, the method recovered certifiably near-optimal solutions after rounding and refinement. Across these problem classes, the implementation reproduced the behavior of hand-designed certifiable estimators while reducing the implementation burden from problem-specific solver design to lifted factor and variable definitions.

## Why this matters for GTSAM

Certifiable factor graph optimization suggests a path for bringing certificates into the software stack roboticists already use. Instead of choosing between a convenient local factor graph model and a separate bespoke certifiable solver, the same model could support both fast local optimization and certificate-producing lifted optimization.

This fits naturally with GTSAM's recent [QP and QCQP support]({% post_url 2026-05-13-qp-qcqp-in-gtsam %}). QCQPs provide the algebraic bridge from factor graph estimation problems to semidefinite relaxations, while GTSAM already provides the factor graph abstraction, sparse optimization machinery, and many of the variable and factor types used in robotics. Our implementation in the paper is built in C++ using GTSAM, and the broader goal is to make certifiable estimation a reusable part of the factor graph workflow rather than a separate custom project for each new problem.

## Further browsing

- [Frontiers of Optimization for Robotics workshop](https://sites.google.com/robotics.utias.utoronto.ca/icra26-frontiers-optimization/)
- [OpenReview: Certifiable Factor Graph Optimization](https://openreview.net/forum?id=hAtI0KkdAg)
- [Arxiv: Certifiable Factor Graph Optimization](https://arxiv.org/abs/2603.01267)
- [Post: Certifiable Optimization at the ICRA Workshop]({% post_url 2026-06-01-icra-for-workshop %})
- [Post: Quadratic Programs and QCQPs in GTSAM]({% post_url 2026-05-13-qp-qcqp-in-gtsam %})

_Disclosure: AI was used to help draft this post and prepare the figures._
