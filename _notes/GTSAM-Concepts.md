---
layout: gtsam-note
title: "GTSAM Concepts"
permalink: /notes/gtsam-concepts/
date: 2019-05-18 14:09:48 -0400
categories: roadmap
---

As discussed in [Generic Programming Techniques](http://www.boost.org/community/generic_programming.html), concepts define:

- associated types
- valid expressions
- invariants
- complexity guarantees

Below are the most important concepts used in GTSAM and how they are expressed through traits.

## Manifold

To optimize over continuous types, GTSAM assumes they are manifolds. We are interested in differentiable manifolds: spaces that can be locally approximated at any point using a tangent space. A chart is an invertible map from the manifold to that tangent space.

In GTSAM, the required operations for a type are defined through specialization of `gtsam::traits`.

Important members include:

- `dimension`: the dimensionality of the manifold
- `TangentVector`: the type that lives in tangent space
- `ChartJacobian`: typically `OptionalJacobian<dimension, dimension>`
- `ManifoldType`: a pointer back to the type
- `structure_category`: one of the tag types that indicates which concept requirements are satisfied

Important operations include:

- `traits<T>::GetDimension(p)`
- `traits<T>::Local(p, q)`
- `traits<T>::Retract(p, v)`

Expected invariants:

- `Retract(p, Local(p, q)) == q`
- `Local(p, Retract(p, v)) == v`

## Group

A [group](https://en.wikipedia.org/wiki/Group_(mathematics)) provides a composition operation that is closed, associative, has an identity element, and has an inverse for each element.

Important operations include:

- `traits<T>::Compose(p, q)`
- `traits<T>::Inverse(p)`
- `traits<T>::Between(p, q)`

Important static members include:

- `traits<T>::Identity`
- `traits<T>::group_flavor`

Expected invariants:

- `Compose(p, Inverse(p)) == Identity`
- `Compose(p, Between(p, q)) == q`
- `Between(p, q) == Compose(Inverse(p), q)`

## Lie Group

A Lie group is both a manifold and a group. In addition to the manifold and group requirements, GTSAM also needs derivative information for composition and inverse.

That means traits for a Lie group need to support the corresponding Jacobian-aware operations used by optimization code.
