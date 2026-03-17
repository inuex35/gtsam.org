---
layout: gtsam-post
title:  "The Manifold Kalman Filter Hierarchy, Part 2: Legged State Estimation"
---

Authors: [Frank Dellaert](https://dellaert.github.io/), Scott Baker

<!-- - TOC -->
{:toc}

[Part I](/2026/03/09/manifold-kf-part1.html) introduced the new manifold and Lie-group Kalman filter hierarchy in GTSAM. This post is about a much more concrete question: what do those ideas buy you on a real legged robot climbing stairs with an IMU and contact kinematics?

The answer is: quite a lot. The newly merged legged-estimation examples in GTSAM show a clean progression from an invariant filter, to an invariant filter with a local graph update, to two fixed-lag smoothers that use the same contact model but retain more information over time.

Early on, the best place to explore all of this is the published notebook [LeggedEstimator.ipynb](https://borglab.github.io/gtsam/leggedestimator/). For source, start with the C++ replay driver [LeggedEstimatorReplayExample.cpp](https://github.com/borglab/gtsam/blob/develop/examples/LeggedEstimatorReplayExample.cpp) and the implementation in [LeggedEstimator.h](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/LeggedEstimator.h).

<figure class="center" style="width: 110%; max-width: 110%; margin-left: -5%;">
  <img src="/assets/images/legged-kf/stairs_side.gif"
    alt="Legged robot staircase estimation using IMU and contact measurements"
    style="width: 100%; max-width: none;" />
  <figcaption>A staircase sequence from the legged-estimation tutorial. The data source is the <a href="https://github.com/SangwooJung98/Co-RaL-Dataset">Co-RaL dataset</a>, and the animation is generated from the corresponding GTSAM notebook/example.</figcaption>
</figure>
<br />

## 1. A simplified invariant filter is already very useful

The first useful legged variant keeps the invariant-filter structure of the earlier tutorial, but trims Hartley's contact-aided construction down to something easy to read and modify in GTSAM.

The starting point is Ross Hartley et al.'s paper ["Contact-aided invariant extended Kalman filtering for robot state estimation"](https://arxiv.org/abs/1904.09251). The important idea is not just "use contacts", but augment the state with contact landmarks in a way that preserves invariant dynamics. In GTSAM, [`LeggedInvariantEKF`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/LeggedEstimator.h) follows that same spirit with an `ExtendedPose3` state containing rotation, position, velocity, and footholds.

Our variant 1 is deliberately simpler. It treats feet as body-frame contact measurements, keeps only the currently active footholds in the state, and in the reference replay setup never has to manage more than four of them. The replay driver also makes the contact-update schedule explicit: it pushes contact packets into the estimator on touchdown, or after `100 ms` of dead reckoning, whichever comes first. That logic lives in [buildContactReplayPlan](https://github.com/borglab/gtsam/blob/develop/examples/LeggedEstimatorReplayExample.cpp), and it is one of the reasons the example is so readable.

This gives a very practical invariant filter: light-weight, recursive, and geometry-aware, without forcing a full smoothing problem at every step. The published notebook [LeggedEstimator.ipynb](https://borglab.github.io/gtsam/leggedestimator/) shows this as `LeggedInvariantEKF`, and the corresponding C++ replay variant is `invariant_ekf` in [LeggedEstimatorReplayExample.cpp](https://github.com/borglab/gtsam/blob/develop/examples/LeggedEstimatorReplayExample.cpp).

## 2. The next step is a graph fragment, not a whole graph

Because GTSAM is a factor-graph library, the next natural step is to replace the pure EKF contact correction with a small nonlinear solve over the current base state and all feet in contact.

That is exactly what [`LeggedInvariantIEKF`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/LeggedEstimator.h) does. The prediction side still uses the same invariant state representation as variant 1, but the measurement phase becomes an iterated local graph update. In other words, the estimator still behaves like a filter overall, yet each contact event is solved more like a tiny factor graph than a single linearized EKF correction.

<figure class="center" style="width: 92%; max-width: 92%;">
  <img src="/assets/images/legged-kf/legged-invariant-iekf-fragment.svg"
    alt="Factor-graph fragment for the invariant IEKF contact update"
    style="width: 100%;" />
  <figcaption>The local graph fragment used in the invariant graph-update variant: a predicted base state on the left, active footholds on the right, and one contact factor per foot currently in stance.</figcaption>
</figure>
<br />

This is a very GTSAM move. Rather than abandoning filtering altogether, variant 2 uses just enough factor-graph machinery to make the contact update richer and more nonlinear. The notebook [LeggedEstimator.ipynb](https://borglab.github.io/gtsam/leggedestimator/) calls this `LeggedInvariantIEKF`, and the replay source exposes it as `invariant_graph` in [LeggedEstimatorReplayExample.cpp](https://github.com/borglab/gtsam/blob/develop/examples/LeggedEstimatorReplayExample.cpp).

## 3. Once the update is a graph problem, a smoother is the natural next step

Once the contact update becomes a graph problem, the natural next step is to keep a short history and smooth instead of throwing information away immediately.

That is what variant 3 does. [`LeggedFixedLagSmoother`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/LeggedEstimator.h) builds a fixed-lag window over contact events, creates foothold variables per contact episode, and links consecutive base states with preintegrated IMU motion factors. In the current implementation those motion links are `ImuFactor2` factors from [ImuFactor.h](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/ImuFactor.h), which is exactly the right level of detail to mention here: the smoother is still using the invariant contact structure, but now it can use several steps of information at once.

<figure class="center" style="width: 112%; max-width: 112%; margin-left: -6%;">
  <img src="/assets/images/legged-kf/legged-fixed-lag-window.svg"
    alt="Fixed-lag smoother structure for the legged estimator"
    style="width: 100%; max-width: none;" />
  <figcaption>Two views of the same progression: variant 3 keeps a sliding window with one shared bias variable, while variant 4 upgrades that to a bias trajectory across the window.</figcaption>
</figure>
<br />

This is where the factor-graph viewpoint really starts to pay off. The filter variants summarize the past into one Gaussian belief at the current time. The smoother instead keeps a short recent history alive, which lets it combine delayed contact information with the accumulated IMU evidence between events. In the notebook [LeggedEstimator.ipynb](https://borglab.github.io/gtsam/leggedestimator/), this appears as `LeggedFixedLagSmoother`, and in the replay driver it is `fixed_lag_single_bias` in [LeggedEstimatorReplayExample.cpp](https://github.com/borglab/gtsam/blob/develop/examples/LeggedEstimatorReplayExample.cpp).

## 4. The last step is to estimate bias over the whole lag, not just once

The last step is to stop assuming a single bias can explain the entire lag window.

Variant 4 keeps the same contact-episode smoother structure, but upgrades the inertial side from one shared bias estimate to a bias trajectory over the window. In GTSAM that means moving from [`LeggedFixedLagSmoother`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/LeggedEstimator.h) to [`LeggedCombinedFixedLagSmoother`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/LeggedEstimator.h), and from `ImuFactor2`-style links to [`CombinedImuFactor`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/CombinedImuFactor.h).

Conceptually, the change is simple but important. Variant 3 says: "within this lag, one bias estimate is good enough." Variant 4 says: "let the bias evolve, and estimate that evolution explicitly." In the replay source this final estimator is the `fixed_lag_combined_bias` variant in [LeggedEstimatorReplayExample.cpp](https://github.com/borglab/gtsam/blob/develop/examples/LeggedEstimatorReplayExample.cpp). That makes it the most expressive of the four reference implementations, and also the closest to the direction many serious legged-estimation systems eventually need to go.

## Closing

This sequence of estimators is a nice example of why the filter hierarchy in Part I was worth building in the first place. The invariant structure gives a strong filtering baseline for contact-aided navigation, the local graph fragment makes the contact update more faithful without giving up online behavior, and the fixed-lag smoothers show how naturally the same ideas extend into fuller factor-graph inference.

By providing these reference implementations for leg robot state estimation, we hope to lower the barrier of entry to using these or implementing your own variants on top of them.

_Disclosure: AI was used to help draft this post._
