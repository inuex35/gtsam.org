---
layout: gtsam-post
title:  "The Manifold Kalman Filter Hierarchy, Part 4: Awesome Equivariant Filters!"
---

*Authors*: [Rohan Bansal](https://rbansal.dev/academic) and [Frank Dellaert](https://dellaert.github.io)

<!-- - TOC -->
{:toc}

In [Part 3](https://gtsam.org/2026/04/28/equivariant.html) of the Manifold Filter Hierarchy, we introduced the `EquivariantFilter` template in GTSAM and walked through the geometry that makes EqF useful: a state on a manifold $\mathcal{M}$, a symmetry group $\mathcal{G}$ acting on it, and an error expressed around a fixed reference state instead of the current estimate. In that post, we discussed a very simple toy problem of an attitude-on-a-sphere.

In today's post, we consider more complex problems which can be made somewhat simpler through the usage of an EqF, such as VIO (Visual-Inertial-Odometry) and ABC (Attitude-Bias-Calibration). In tandem, we also announce **[AwesomeEqF](https://borglab.github.io/AwesomeEqF/)**: a community-curated collection of papers and runnable notebooks for equivariant filtering, built around GTSAM.

## What is AwesomeEqF?

<figure class="center" style="width: 100%; max-width: 700px;">
  <img src="/assets/images/awesome-eqf/header_figure.png"
    alt="AwesomeEqF header figure."
    style="width: 100%;" />
</figure>
<br />

AwesomeEqF is [website](https://borglab.github.io/AwesomeEqF/) (and [repo](https://github.com/borglab/AwesomeEqF)) containing:

- A reading list of papers, organized from foundational invariant-observer papers through to recent EqF architectures
- A growing set of notebooks that utilize GTSAM's EqF filter implementation and perform on real data
- A soon-to-be blog for tutorials and write-ups that contextualize specific papers.

The intent is for AwesomeEqF to be the place a roboticist lands when they have read about the potential of equivariant filtering and are excited to get their hands dirty. Contributions are welcome, see the [Contributing Guide](https://github.com/borglab/AwesomeEqF/blob/main/CONTRIBUTING.md).

## A Quick Recap of the EqF

From Part 3: an EqF stores the estimate as a fixed reference state $\xi^\circ \in \mathcal{M}$ together with a group element $\hat{g} \in \mathcal{G}$. The current estimate is recovered by the group action $\hat{\xi}$, and the natural error $e$ lands at $\xi^\circ$ when the filter is correct.

$$
\hat{\xi} = \phi(\hat{g}, \xi^\circ),
$$

$$
e = \phi(\hat{g}^{-1}, \xi)
$$

Because the linearization happens around that fixed reference rather than the moving estimate, covariance propagation depends much less on whether the current guess is right.

An important thing to note is that for any new EqF, we also need to supply the **lift** that turns physical dynamics into a small motion in the group, and the **equivariance conditions** that the dynamics and outputs have to satisfy. Once those are in, the actual runtime loop looks like an ordinary EKF on the group tangent space.

Now we will jump into two practical use cases of the EqF, in the trajectory state estimation case and the attitude estimation case.

## EqVIO: Equivariant Visual-Inertial Odometry

The **EqVIO** by **[Pieter van Goor and Robert Mahony](https://arxiv.org/abs/2205.01980)** is the answer to a long-standing complaint about EKF-based VIO: standard filters such as MSCKF (Multi-State Constraint Kalman Filter) accumulate inconsistency because the linearization point drifts with the (possibly wrong) estimate, and the filter ends up more confident than its actual error warrants. EqVIO removes that source of inconsistency by construction, using equivariance.

### The VIO state

At every timestep, EqVIO's physical state is composed of:

1. body pose $P \in SE(3)$,
3. body linear velocity $v \in \mathbb{R}^3$,
4. gyroscope bias $b_\omega \in \mathbb{R}^3$,
5. accelerometer bias $b_a \in \mathbb{R}^3$,
6. camera-to-IMU rigid offset $T_{ci} \in SE(3)$,
7. variable-size set of 3D landmarks $p_i \in \mathbb{R}^3$ corresponding to tracked image features.

<br />
<figure class="center" style="width: 100%; max-width: 700px;">
  <img src="/assets/images/awesome-eqf/eqvio_state.png"
    alt="EqVIO state diagram."
    style="width: 100%;" />
  <figcaption>Figure from <a href="https://arxiv.org/pdf/2205.01980">van Goor et. al</a>, visualizing the state of the system relative to the origin.</figcaption>
</figure>
<br />

Note that the state is *not* a Lie group, but rather a manifold, which is exactly the situation an EqF is good for.

### The symmetry group

EqVIO's contribution is identifying a symmetry group $\mathcal{G}$ that acts transitively on this composite VIO state. The group is built as a product Lie group whose factors mirror the state itself:

$$
\mathcal{G} = SE_2(3) \ltimes \mathbb{R}^6 \times SE(3) \times \prod_i \mathrm{SOT}(3).
$$

The intuition behind the pieces:

- **$SE_2(3)$**, the "extended pose" group, jointly handles attitude, position, and velocity.
- **$\mathbb{R}^6$** for the two IMU biases, which the semi-direct product couples back into the inertial frame.
- **$SE(3)$** for the camera offset $T_{ci}$, so online intrinsic-extrinsic refinement is part of the geometry.
- **$\mathrm{SOT}(3)$** (rotation plus scaling) per landmark, capturing the rotation-and-depth ambiguity that visual measurements really do have.

The action $\phi$ of $\mathcal{G}$ on the state is then the natural one inherited from each factor.

**What is $\mathrm{SOT}(3)$ though?** Some readers may rightly question what this group is; it is not a commonly defined group in GTSAM, but rather specific to the EqVIO implementation. Concretely, 

$$
\mathrm{SOT}(3) = SO(3) \times \mathbb{R}_{>0}
$$ 

is the direct product of a rotation $R \in SO(3)$ and a strictly positive scalar $s \in \mathbb{R}_{>0}$, acting on a 3D point as $q \mapsto s\, R\, q$. This group matters because monocular projection only fixes a landmark up to its bearing direction and a positive depth scaling, which is exactly the ambiguity that $\mathrm{SOT}(3)$ encodes, so making it part of the symmetry is what lets the equivariant output approximation in the next section work.

### The equivariant output approximation

One of the more important practical ideas in the EqVIO paper is the **equivariant output approximation** for the camera measurement. 

We can say that the camera measurement is "equivariant" if applying a transformation to the state results in a predictable transformation of the measurement. As is classic for VIO systems, the camera observes a landmark and produces a bearing vector. Van Goor and Mahony prove that, with respect to the $SOT(3)$ component of their group, this measurement function is equivariant.

A significant side-effect is that this largely improves how the filter processes input camera data. Traditionally, EKFs use a first-order approximation of the measurement function; by exploiting the equivariance property, we can use the group structure to apply an equivariant output approximation and reduce the measurement error by an order of magnitude.

Additionally, this proof also justifies the inverse-depth parameterization. It is [well known](https://en.wikipedia.org/wiki/Inverse_depth_parametrization) that representing landmarks by inverse distance can improve stability of a filter. The authors present a polar parameterization derived from $\mathrm{SOT}(3)$ that can act similar to inverse-depth but also further minimize linearization error. Currently, this polar parameterization is not implemented into GTSAM's EqVIO, but it is a near-future change on the roadmap.

### What the runtime loop looks like

After reading the first three blog posts, the following steps are probably familiar:

1. **Predict.** IMU frames drive a small motion in the group via the lift.
2. **Update.** For each tracked feature in a new image frame, predict the normalized image coordinate, compare to the observation, form the equivariant innovation, and apply a correction in the tangent space.
3. **Manage features.** Add new landmarks for unseen features, drop features that fall out of the image, and keep the covariance block consistent with the live landmark set.

That last step is new, but it is essential bookkeeping for letting the VIO filter run for as long as possible on a real sequence without the state vector blowing up.

## EqVIO in GTSAM

GTSAM's implementation lives in `gtsam_unstable/navigation`:

- [`EqVIOState.h`](https://github.com/borglab/gtsam/blob/develop/gtsam_unstable/navigation/EqVIOState.h): the manifold state described above, including the dynamic landmark list.
- [`EqVIOSymmetry.h`](https://github.com/borglab/gtsam/blob/develop/gtsam_unstable/navigation/EqVIOSymmetry.h): the semi-direct-product group $\mathcal{G}$, its action on `EqVIOState`, and the lift used during prediction.
- [`EqVIOFilter.h`](https://github.com/borglab/gtsam/blob/develop/gtsam_unstable/navigation/EqVIOFilter.h): the filter class and the `predict` / `update` entry points.

`EqVIOFilter` is built on the `EquivariantFilter` template that [Part 3](https://gtsam.org/2026/04/28/equivariant.html) introduced. `xi_ref_` is the EqVIO reference state, and `g_` is the lifted group estimate that the IMU drives.

The filter is also made available in Python through `gtsam_unstable.eqvio` bindings, which mirrors the C++ API very closely. The below snippet walks through the initialization and propagation of the filter, whcih is described in more detail in the AwesomeEqF notebook!

```python
import gtsam, gtsam_unstable

xi_ref = gtsam_unstable.eqvio.State()
xi_ref.cameraOffset = T_ci
params = gtsam_unstable.eqvio.EqVIOFilterParams()
filter = gtsam_unstable.eqvio.EqVIOFilter(
    xi_ref, initial_covariance, gtsam.KeyVector(), params
)

filter.initializeFromIMU(first_imu)
for imu_input, dt in imu_stream:
    filter.predict(imu_input, dt)
for features, R in vision_stream:  # features: dict[int, np.ndarray(2,)]
    filter.update(features, camera, R)
```

The full Python walkthrough lives [here](https://borglab.github.io/AwesomeEqF/notebooks/eqvio-example/), as a notebook on AwesomeEqF. Take a look!

## ABC-EqF: Attitude, Bias, Calibration

In addition to the EqVIO filter, we will also briefly discuss the **ABC-EqF** by [Fornasier, Ng, Brommer, Böhm, Mahony, and Weiss](https://arxiv.org/abs/2209.12038), which is a paper covering equivariant filter design for attitude state estimation.

The state stacks three pieces:

- attitude $R \in SO(3)$,
- gyroscope bias $b \in \mathbb{R}^3$,
- and a sensor calibration rotation $S \in SO(3)$ that aligns a reference direction sensor (e.g. magnetometer) with the body frame.

The classical EKF treats bias and calibration as ordinary linear states tacked onto attitude, but the ABC-EqF instead builds a single symmetry group $SO(3) \times \mathbb{R}^3 \times SO(3)$ that natively contains the bias and calibration as part of the geometric structure, so the bias-aware error and the calibration-aware error transport correctly under the group action. The result is better linearization behavior, faster bias convergence, and an estimator whose consistency does not depend on the bias estimate being good.

In GTSAM this is implemented as the [`ABCEquivariantFilter`](https://github.com/borglab/gtsam/blob/develop/gtsam_unstable/geometry/ABCEquivariantFilter.h) in `gtsam_unstable`. A complete C++ example lives at [`AbcEquivariantFilterExample.cpp`](https://github.com/borglab/gtsam/blob/develop/examples/AbcEquivariantFilterExample.cpp), and the AwesomeEqF notebook [here](https://borglab.github.io/AwesomeEqF/notebooks/abc-eqf-example/) walks through that code in Python with interactive plots for the attitude, bias, and calibration errors over time.

## Takeaway

The EqVIO filter is the most recent application of the EqF machinery from Part 3, and it is now usable from Python through GTSAM. 

Additionally, AwesomeEqF is meant to grow with the field! If you have a paper, a notebook, or a write-up that fits, please open a pull request.

## Further Reading

- ["EqVIO: An Equivariant Filter for Visual Inertial Odometry"](https://arxiv.org/abs/2205.01980), van Goor and Mahony.
- ["Overcoming Bias: Equivariant Filter Design for Biased Attitude Estimation with Online Calibration"](https://arxiv.org/abs/2209.12038), Fornasier, Ng, Brommer, Böhm, Mahony, and Weiss.
- ["Equivariant Symmetries for Aided Inertial Navigation"](https://arxiv.org/abs/2407.14297), Fornasier (dissertation).
- AwesomeEqF site: [borglab.github.io/AwesomeEqF](https://borglab.github.io/AwesomeEqF/).
- GTSAM EqVIO source: [`gtsam_unstable/navigation/EqVIOFilter.h`](https://github.com/borglab/gtsam/blob/develop/gtsam_unstable/navigation/EqVIOFilter.h).
- AwesomeEqF EqVIO notebook source: [`02_eqvio_example.ipynb`](https://github.com/borglab/AwesomeEqF/blob/master/notebooks/02_eqvio_example.ipynb).
- AwesomeEqF ABC-EqF notebook source: [`01_abc_eqf_example.ipynb`](https://github.com/borglab/AwesomeEqF/blob/master/notebooks/01_abc_eqf_example.ipynb).
