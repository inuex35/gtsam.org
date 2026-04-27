---
layout: gtsam-post
title:  "The Manifold Kalman Filter Hierarchy, Part 3: Equivariant Filters"
---

> **Authors:** Rohan Bansal, Jennifer Oum, and [Frank Dellaert](https://dellaert.github.io/)

<!-- - TOC -->
{:toc}

Last week we zoomed out and talked about [STAG](/2026/04/21/factor-graphs-and-world-models.html): state, dynamics, measurements, objectives, and how factor graphs can tie those pieces together. This week we zoom back in on one very GTSAM-specific corner of that story: if the state is not a vector space, what kind of Kalman filter should carry the online belief?

This is the third post in the manifold Kalman filter hierarchy series. [Part 1](/2026/03/09/manifold-kf-part1.html) introduced:

- `ManifoldEKF`
- `LieGroupEKF`
- `InvariantEKF`
- `LeftLinearEKF`

[Part 2](/2026/03/17/legged-state-estimation-part2.html) showed why invariant filtering matters in legged state estimation. The key advantage there was not only that the state lives on a Lie group. It was that the error dynamics can be made independent of the current state estimate, so uncertainty propagation remains meaningful even when the estimate is wrong.

That leaves an important gap. `ManifoldEKF` can already handle states such as `Unit3`, where the state is a point on a sphere rather than a Lie group. `InvariantEKF` gives us the more robust invariant-error behavior, but it assumes the state itself is a group. The `EquivariantFilter` fills that gap: the physical state can live on a general manifold, while a separate symmetry group acts on that state and gives us the error structure we wanted from invariant filtering.

<figure class="center" style="width: 100%; max-width: 860px;">
  <img src="/assets/images/equivariant/filter-hierarchy-gap.png"
    alt="The GTSAM Kalman filter hierarchy showing EquivariantFilter as the branch for manifold states with a symmetry group action"
    style="width: 100%;" />
  <figcaption>EqF keeps the physical state on the manifold, but uses a symmetry group action to define and propagate the error.</figcaption>
</figure>
<br />

If you want the full theory, a good starting point is the equivariant systems tutorial by Mahony, Hamel, and Trumpf, ["Equivariant Systems Theory and Observer Design"](https://arxiv.org/abs/2006.08276), and the EqF paper ["Equivariant Filter Design for Kinematic Systems on Lie Groups"](https://arxiv.org/abs/2010.14666) by van Goor, Hamel, and Mahony. This post is the GTSAM-user version: what problem is the class solving, and what do you need to provide?

## The Gap in the Hierarchy

The shortest possible summary is:

> `EquivariantFilter` is an error-state Kalman filter where the error is chosen using a symmetry action, even when the physical state is not itself a group.

That sentence is dense, so let us compare the relevant filters.

`ManifoldEKF` is the broadest base class. It only needs a manifold state `M`: something with local coordinates, `retract`, and `localCoordinates`. That works for `Pose3`, `Rot3`, `NavState`, ordinary vectors, and also for `Unit3`, which represents a direction on the unit sphere. The catch is that the covariance and linearized dynamics are tied to the current estimate. That is the standard EKF story on a manifold.

`InvariantEKF` is more specialized. It assumes the state is a Lie group and uses group structure to define an invariant error. When the dynamics have the right form, the propagated error no longer depends on the current estimate. This is why invariant filters are attractive at startup or during large transients: a bad estimate does not immediately make the covariance propagation bad in the same way a standard EKF linearization can.

`EquivariantFilter` sits between those two ideas. The state can be a manifold that is not a group. But we still introduce a group that moves points around on that manifold. That group action lets the filter define an error around a fixed reference point, rather than continually redefining everything around the current state estimate.

In other words, EqF lets us have our cake and eat it too:

- the physical state can be a general manifold `M`
- the uncertainty propagation can exploit a symmetry group `G`
- the usual EKF machinery still handles prediction, Kalman gain, and measurement update

## A Simple Example

Imagine a robot carrying a gyroscope and a magnetometer. We want to estimate the direction of the magnetic field in the robot's body frame. That direction is a unit vector

$$
\eta \in \mathbb{S}^2,
$$

with dynamics

$$
\dot{\eta} = -\Omega^\times \eta,
$$

and measurement

$$
y = c_m \eta.
$$

Here, $\Omega$ is the angular velocity measured by the gyroscope. The matrix $\Omega^\times$ is the cross-product matrix, so the dynamics say that the magnetic-field direction appears to rotate in the robot's body frame as the robot turns. The measurement equation says the magnetometer observes that same direction, scaled by a constant magnetic-field strength $c_m$.

The important detail is that the state is not a group. It is a point on the sphere:

$$
M = \mathbb{S}^2.
$$

In GTSAM, that is a `Unit3`. You can perturb it locally with two tangent coordinates, but you cannot compose two directions and get another direction in the group-theoretic sense. So this is a natural state for `ManifoldEKF`, not for `InvariantEKF`.

However, rotations act on directions. The group

$$
G = SO(3)
$$

moves points around on $\mathbb{S}^2$. In GTSAM terms, the physical state can be `Unit3`, while the symmetry group can be `Rot3`. This is exactly the kind of situation EqF is designed for: the state is not a group, but a group still acts on it.

## Manifolds, Groups, and Actions

There are three geometric objects in play.

A **manifold** is the physical state space. It might be curved globally, even though it has local tangent coordinates. `Unit3`, `Rot3`, `Pose3`, and `NavState` are all manifolds in GTSAM.

A **group** is something we can compose, invert, and compare to an identity element. `Rot3` and `Pose3` are groups; `Unit3` is not.

A **group action** is a rule for how a group element moves a state. For the sphere example:

- the manifold state is a direction $\eta \in \mathbb{S}^2$
- the group is $SO(3)$
- the action rotates the direction

This is the extra structure beyond `ManifoldEKF`. EqF does not require the state to be a group. It requires a useful group action on the state.

## The EqF Idea

Instead of storing the estimate directly as a state $\hat{\xi}$ on the manifold, the EqF stores a group element $\hat{X}$ and applies it to a fixed reference state $\xi^\circ$:

$$
\hat{\xi} = \phi(\hat{X}, \xi^\circ).
$$

Here, $\phi$ is the group action. In the sphere example, $\xi^\circ$ might be the north pole direction, and $\hat{X}$ is a rotation that moves that reference direction to the current estimate.

This also gives a natural error:

$$
e = \phi(\hat{X}^{-1}, \xi).
$$

If the estimate is correct, the error lands back at the fixed reference point:

$$
e = \xi^\circ.
$$

That fixed reference point is the key. In a usual manifold EKF, the tangent space and linearization move with the current estimate. In an equivariant filter, the symmetry action lets us express the error around a fixed origin. With the right equivariance built into the dynamics and outputs, the local error model is much less sensitive to the current estimate being wrong.

The runtime loop still looks like an EKF: predict the state, propagate covariance, compare predicted and observed measurements, compute a Kalman gain, and apply a correction. The difference is where the estimate and correction live. The physical estimate lives on the manifold, but prediction and correction are mediated by the group action.

During prediction, the **lift** converts physical dynamics into a small motion in the group. In the attitude-direction example, the gyroscope tells us how to move the group estimate, and the direction estimate follows from the group action. During update, the Kalman correction is lifted back through the group instead of directly nudging arbitrary coordinates on the sphere.

## What the User Provides

The price of the extra structure is that the model has to expose the right symmetries. For a concrete EqF, the user provides:

- the physical state manifold `M`
- the symmetry action, encoded as a `Symmetry` functor
- a lift from physical dynamics to group motion
- the input action needed to express the dynamics equivariantly
- output or measurement models with the corresponding equivariance
- the usual process and measurement noise models

This is a little more work than a plain `ManifoldEKF`, where you can often just provide the next state and a transition Jacobian. The payoff is that, when the symmetry is real, the filter is using the geometry of the problem rather than an arbitrary local coordinate choice.

## What It Looks Like in GTSAM

The generic implementation lives in [`gtsam/navigation/EquivariantFilter.h`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/EquivariantFilter.h).

It is templated on:

- `M`, the physical manifold state type
- `Symmetry`, the functor that defines the group action

The class inherits from `ManifoldEKF<M>`, so it reuses the same covariance, Kalman gain, and Joseph-form update machinery from the base hierarchy. The EqF-specific part is the additional symmetry bookkeeping.

In the implementation:

- `Symmetry::Group` is the group `G` used internally
- `xi_ref_` is the fixed reference state $\xi^\circ$
- `g_` is the lifted group estimate
- `act_on_ref_(g_)` recovers the current manifold estimate
- `Dphi0_` is the differential of the action at the identity
- `InnovationLift_` maps a small manifold correction back to a group tangent correction

The constructor takes the reference state and initial covariance:

```cpp
EquivariantFilter(const M& xi_ref, const CovarianceM& Sigma,
                  const G& X0 = traits<G>::Identity())
```

It initializes the base `ManifoldEKF` at `xi_ref`, precomputes the action differential, computes the pseudo-inverse used for the innovation lift, and then sets the actual state estimate by applying the group estimate to the reference state.

Prediction has two modes. The automatic mode uses a `Lift` and an `InputOrbit` to compute the error dynamics matrix:

```cpp
filter.predict(lift_u, psi_u, Qc, dt);
```

There is also an explicit mode when the user wants to provide the Jacobian directly:

```cpp
filter.predictWithJacobian(lift_u, A, Qc, dt);
```

The measurement update keeps the familiar EKF shape. You can pass a measurement function, an observation, and a covariance, and the filter handles the correction through the group estimate:

```cpp
filter.update(h, z, R);
```

So the user-facing API still feels like the rest of the hierarchy, but the estimate is represented internally by both a physical manifold state and a lifted group state.

## When To Reach For It

EqF is worth considering when all three of these are true:

1. Your state lives naturally on a manifold that is not necessarily a group.
2. A Lie group acts on that state in a meaningful way.
3. You care about the invariant-filter benefit: error and covariance propagation that are less dependent on the current estimate being right.

The sphere example is the small version: `Unit3` is not a group, but `Rot3` acts on it. Visual SLAM and other homogeneous-space problems are more ambitious versions of the same idea.

If the state is just a generic manifold with no useful symmetry, `ManifoldEKF` is the right abstraction. If the state is a Lie group and the dynamics have the invariant form, `InvariantEKF` or `LeftLinearEKF` might be simpler. EqF is for the middle case: not a group state, but still a symmetry-rich state.

## Takeaways

`EquivariantFilter` is not an unrelated extra filter. It is the natural next step after the hierarchy in Part 1 and the invariant-filter application in Part 2.

The hierarchy now has a better answer for states such as directions on a sphere: keep the state on the physical manifold, but let a symmetry group push that state around. That gives GTSAM users a way to model states that are not groups without giving up the robustness motivation behind invariant filtering.

As a GTSAM user, the main question is no longer only "is my state a manifold?" or "is my state a group?" The EqF question is:

> what group acts naturally on my state, and can I write the dynamics and measurements in a way that respects that action?

If the answer is yes, `EquivariantFilter<M, Symmetry>` gives you a reusable template for the rest of the EKF machinery.

## Further Reading

- ["Equivariant Systems Theory and Observer Design"](https://arxiv.org/abs/2006.08276), Mahony, Hamel, and Trumpf
- ["Equivariant Filter Design for Kinematic Systems on Lie Groups"](https://arxiv.org/abs/2010.14666), van Goor, Hamel, and Mahony
- GTSAM EqF test: [`testEquivariantFilter.cpp`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/tests/testEquivariantFilter.cpp)
- More complicated ABC EqF example in GTSAM: [Example](https://github.com/borglab/gtsam/blob/develop/examples/AbcEquivariantFilterExample.cpp), [Filter](https://github.com/borglab/gtsam/blob/develop/gtsam_unstable/geometry/ABCEquivariantFilter.h)

_Disclosure: AI was used to help draft this post._
