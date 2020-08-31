/*
 * This file is part of React, licensed under the MIT License (MIT).
 *
 * Copyright (c) 2013 Flow Powered <https://flowpowered.com/>
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://danielchappuis.ch>
 * React is re-licensed with permission from ReactPhysics3D author.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
package com.flowpowered.react.collision.narrowphase.EPA;

import units.qual.Dimensionless;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.Queue;

import com.flowpowered.react.ReactDefaults;
import com.flowpowered.react.collision.narrowphase.GJK.GJKAlgorithm;
import com.flowpowered.react.collision.narrowphase.GJK.Simplex;
import com.flowpowered.react.collision.shape.CollisionShape;
import com.flowpowered.react.constraint.ContactPoint.ContactPointInfo;
import com.flowpowered.react.math.Matrix3x3;
import com.flowpowered.react.math.Quaternion;
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector3;

/**
 * This class is the implementation of the Expanding Polytope Algorithm (EPA). The EPA algorithm computes the penetration depth and contact points between two enlarged objects (with margin) where the
 * original objects (without margin) intersect. The penetration depth of a pair of intersecting objects A and B is the length of a point on the boundary of the Minkowski sum (A-B) closest to the
 * origin. The goal of the EPA algorithm is to start with an initial simplex polytope that contains the origin and expend it in order to find the point on the boundary of (A-B) that is closest to the
 * origin. An initial simplex that contains the origin has been computed with the GJK algorithm. The EPA Algorithm will extend this simplex polytope to find the correct penetration depth. The
 * implementation of the EPA algorithm is based on the book "Collision Detection in 3D Environments".
 */
@Dimensionless
public class EPAAlgorithm {
    private static final @Dimensionless int MAX_SUPPORT_POINTS = ((@Dimensionless int) (100));
    private static final @Dimensionless int MAX_FACETS = ((@Dimensionless int) (200));

    /**
     * Computes the penetration depth with the EPA algorithm. This method computes the penetration depth and contact points between two enlarged objects (with margin) where the original objects
     * (without margin) intersect. An initial simplex that contains the origin has been computed with GJK algorithm. The EPA Algorithm will extend this simplex polytope to find the correct penetration
     * depth. Returns true if the computation was successful, false if not.
     *
     * @param simplex The initial simplex
     * @param collisionShape1 The first collision shape
     * @param transform1 The transform of the first collision shape
     * @param collisionShape2 The second collision shape
     * @param transform2 The transform of the second collision shape
     * @param v The vector in which to store the closest point
     * @param contactInfo The contact info in which to store the contact info of the collision
     * @return Whether or not the computation was successful
     */
    public @Dimensionless boolean computePenetrationDepthAndContactPoints(@Dimensionless EPAAlgorithm this, @Dimensionless Simplex simplex,
                                                           @Dimensionless
                                                           CollisionShape collisionShape1, @Dimensionless Transform transform1,
                                                           @Dimensionless
                                                           CollisionShape collisionShape2, @Dimensionless Transform transform2,
                                                           @Dimensionless
                                                           Vector3 v, @Dimensionless ContactPointInfo contactInfo) {
        final @Dimensionless Vector3 @Dimensionless [] suppPointsA = new Vector3 @Dimensionless [MAX_SUPPORT_POINTS];
        final @Dimensionless Vector3 @Dimensionless [] suppPointsB = new Vector3 @Dimensionless [MAX_SUPPORT_POINTS];
        final @Dimensionless Vector3 @Dimensionless [] points = new Vector3 @Dimensionless [MAX_SUPPORT_POINTS];
        final @Dimensionless TrianglesStore triangleStore = new @Dimensionless TrianglesStore();
        final @Dimensionless Queue<@Dimensionless TriangleEPA> triangleHeap = new @Dimensionless PriorityQueue<>(MAX_FACETS, new @Dimensionless TriangleComparison());
        final @Dimensionless Transform body2Tobody1 = Transform.multiply(transform1.getInverse(), transform2);
        final @Dimensionless Matrix3x3 rotateToBody2 = Matrix3x3.multiply(transform2.getOrientation().getMatrix().getTranspose(), transform1.getOrientation().getMatrix());
        @Dimensionless
        int nbVertices = simplex.getSimplex(suppPointsA, suppPointsB, points);
        final @Dimensionless float tolerance = ReactDefaults.MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint();
        @Dimensionless
        int nbTriangles = ((@Dimensionless int) (0));
        triangleStore.clear();
        switch (nbVertices) {
            case ((@Dimensionless int) (1)):
                return false;
            case ((@Dimensionless int) (2)): {
                final @Dimensionless Vector3 d = Vector3.subtract(points[((@Dimensionless int) (1))], points[((@Dimensionless int) (0))]).getUnit();
                final @Dimensionless int minAxis = d.getAbsoluteVector().getMinAxis();
                final @Dimensionless float sin60 = (@Dimensionless float) Math.sqrt(((@Dimensionless int) (3))) * ((@Dimensionless float) (0.5f));
                final @Dimensionless Quaternion rotationQuat = new @Dimensionless Quaternion(d.getX() * sin60, d.getY() * sin60, d.getZ() * sin60, ((@Dimensionless float) (0.5f)));
                final @Dimensionless Matrix3x3 rotationMat = rotationQuat.getMatrix();
                final @Dimensionless Vector3 v1 = d.cross(new @Dimensionless Vector3(minAxis == ((@Dimensionless int) (0)) ? ((@Dimensionless int) (1)) : ((@Dimensionless int) (0)), minAxis == ((@Dimensionless int) (1)) ? ((@Dimensionless int) (1)) : ((@Dimensionless int) (0)), minAxis == ((@Dimensionless int) (2)) ? ((@Dimensionless int) (1)) : ((@Dimensionless int) (0))));
                final @Dimensionless Vector3 v2 = Matrix3x3.multiply(rotationMat, v1);
                final @Dimensionless Vector3 v3 = Matrix3x3.multiply(rotationMat, v2);
                suppPointsA[((@Dimensionless int) (2))] = collisionShape1.getLocalSupportPointWithMargin(v1);
                suppPointsB[((@Dimensionless int) (2))] = Transform.multiply(
                        body2Tobody1,
                        collisionShape2.getLocalSupportPointWithMargin(Matrix3x3.multiply(rotateToBody2, Vector3.negate(v1))));
                points[((@Dimensionless int) (2))] = Vector3.subtract(suppPointsA[((@Dimensionless int) (2))], suppPointsB[((@Dimensionless int) (2))]);
                suppPointsA[((@Dimensionless int) (3))] = collisionShape1.getLocalSupportPointWithMargin(v2);
                suppPointsB[((@Dimensionless int) (3))] = Transform.multiply(
                        body2Tobody1,
                        collisionShape2.getLocalSupportPointWithMargin(Matrix3x3.multiply(rotateToBody2, Vector3.negate(v2))));
                points[((@Dimensionless int) (3))] = Vector3.subtract(suppPointsA[((@Dimensionless int) (3))], suppPointsB[((@Dimensionless int) (3))]);
                suppPointsA[((@Dimensionless int) (4))] = collisionShape1.getLocalSupportPointWithMargin(v3);
                suppPointsB[((@Dimensionless int) (4))] = Transform.multiply(
                        body2Tobody1,
                        collisionShape2.getLocalSupportPointWithMargin(Matrix3x3.multiply(rotateToBody2, Vector3.negate(v3))));
                points[((@Dimensionless int) (4))] = Vector3.subtract(suppPointsA[((@Dimensionless int) (4))], suppPointsB[((@Dimensionless int) (4))]);
                if (isOriginInTetrahedron(points[((@Dimensionless int) (0))], points[((@Dimensionless int) (2))], points[((@Dimensionless int) (3))], points[((@Dimensionless int) (4))]) == ((@Dimensionless int) (0))) {
                    suppPointsA[((@Dimensionless int) (1))].set(suppPointsA[((@Dimensionless int) (4))]);
                    suppPointsB[((@Dimensionless int) (1))].set(suppPointsB[((@Dimensionless int) (4))]);
                    points[((@Dimensionless int) (1))].set(points[((@Dimensionless int) (4))]);
                } else if (isOriginInTetrahedron(points[((@Dimensionless int) (1))], points[((@Dimensionless int) (2))], points[((@Dimensionless int) (3))], points[((@Dimensionless int) (4))]) == ((@Dimensionless int) (0))) {
                    suppPointsA[((@Dimensionless int) (0))].set(suppPointsA[((@Dimensionless int) (4))]);
                    suppPointsB[((@Dimensionless int) (0))].set(suppPointsB[((@Dimensionless int) (4))]);
                    points[((@Dimensionless int) (0))].set(points[((@Dimensionless int) (4))]);
                } else {
                    return false;
                }
                nbVertices = ((@Dimensionless int) (4));
            }
            case ((@Dimensionless int) (4)): {
                final @Dimensionless int badVertex = isOriginInTetrahedron(points[((@Dimensionless int) (0))], points[((@Dimensionless int) (1))], points[((@Dimensionless int) (2))], points[((@Dimensionless int) (3))]);
                if (badVertex == ((@Dimensionless int) (0))) {
                    final @Dimensionless TriangleEPA face0 = triangleStore.newTriangle(points, ((@Dimensionless int) (0)), ((@Dimensionless int) (1)), ((@Dimensionless int) (2)));
                    final @Dimensionless TriangleEPA face1 = triangleStore.newTriangle(points, ((@Dimensionless int) (0)), ((@Dimensionless int) (3)), ((@Dimensionless int) (1)));
                    final @Dimensionless TriangleEPA face2 = triangleStore.newTriangle(points, ((@Dimensionless int) (0)), ((@Dimensionless int) (2)), ((@Dimensionless int) (3)));
                    final @Dimensionless TriangleEPA face3 = triangleStore.newTriangle(points, ((@Dimensionless int) (1)), ((@Dimensionless int) (3)), ((@Dimensionless int) (2)));
                    if (!(face0 != null && face1 != null && face2 != null && face3 != null
                            && face0.getDistSquare() > ((@Dimensionless int) (0)) && face1.getDistSquare() > ((@Dimensionless int) (0))
                            && face2.getDistSquare() > ((@Dimensionless int) (0)) && face3.getDistSquare() > ((@Dimensionless int) (0)))) {
                        return false;
                    }
                    TriangleEPA.link(new @Dimensionless EdgeEPA(face0, ((@Dimensionless int) (0))), new @Dimensionless EdgeEPA(face1, ((@Dimensionless int) (2))));
                    TriangleEPA.link(new @Dimensionless EdgeEPA(face0, ((@Dimensionless int) (1))), new @Dimensionless EdgeEPA(face3, ((@Dimensionless int) (2))));
                    TriangleEPA.link(new @Dimensionless EdgeEPA(face0, ((@Dimensionless int) (2))), new @Dimensionless EdgeEPA(face2, ((@Dimensionless int) (0))));
                    TriangleEPA.link(new @Dimensionless EdgeEPA(face1, ((@Dimensionless int) (0))), new @Dimensionless EdgeEPA(face2, ((@Dimensionless int) (2))));
                    TriangleEPA.link(new @Dimensionless EdgeEPA(face1, ((@Dimensionless int) (1))), new @Dimensionless EdgeEPA(face3, ((@Dimensionless int) (0))));
                    TriangleEPA.link(new @Dimensionless EdgeEPA(face2, ((@Dimensionless int) (1))), new @Dimensionless EdgeEPA(face3, ((@Dimensionless int) (1))));
                    nbTriangles = addFaceCandidate(face0, triangleHeap, nbTriangles, ((@Dimensionless float) (Float.MAX_VALUE)));
                    nbTriangles = addFaceCandidate(face1, triangleHeap, nbTriangles, ((@Dimensionless float) (Float.MAX_VALUE)));
                    nbTriangles = addFaceCandidate(face2, triangleHeap, nbTriangles, ((@Dimensionless float) (Float.MAX_VALUE)));
                    nbTriangles = addFaceCandidate(face3, triangleHeap, nbTriangles, ((@Dimensionless float) (Float.MAX_VALUE)));
                    break;
                }
                if (badVertex < ((@Dimensionless int) (4))) {
                    suppPointsA[badVertex - ((@Dimensionless int) (1))].set(suppPointsA[((@Dimensionless int) (4))]);
                    suppPointsB[badVertex - ((@Dimensionless int) (1))].set(suppPointsB[((@Dimensionless int) (4))]);
                    points[badVertex - ((@Dimensionless int) (1))].set(points[((@Dimensionless int) (4))]);
                }
                nbVertices = ((@Dimensionless int) (3));
            }
            case ((@Dimensionless int) (3)): {
                final @Dimensionless Vector3 v1 = Vector3.subtract(points[((@Dimensionless int) (1))], points[((@Dimensionless int) (0))]);
                final @Dimensionless Vector3 v2 = Vector3.subtract(points[((@Dimensionless int) (2))], points[((@Dimensionless int) (0))]);
                final @Dimensionless Vector3 n = v1.cross(v2);
                suppPointsA[((@Dimensionless int) (3))] = collisionShape1.getLocalSupportPointWithMargin(n);
                suppPointsB[((@Dimensionless int) (3))] = Transform.multiply(
                        body2Tobody1,
                        collisionShape2.getLocalSupportPointWithMargin(Matrix3x3.multiply(rotateToBody2, Vector3.negate(n))));
                points[((@Dimensionless int) (3))] = Vector3.subtract(suppPointsA[((@Dimensionless int) (3))], suppPointsB[((@Dimensionless int) (3))]);
                suppPointsA[((@Dimensionless int) (4))] = collisionShape1.getLocalSupportPointWithMargin(Vector3.negate(n));
                suppPointsB[((@Dimensionless int) (4))] = Transform.multiply(
                        body2Tobody1,
                        collisionShape2.getLocalSupportPointWithMargin(Matrix3x3.multiply(rotateToBody2, n)));
                points[((@Dimensionless int) (4))] = Vector3.subtract(suppPointsA[((@Dimensionless int) (4))], suppPointsB[((@Dimensionless int) (4))]);
                final @Dimensionless TriangleEPA face0 = triangleStore.newTriangle(points, ((@Dimensionless int) (0)), ((@Dimensionless int) (1)), ((@Dimensionless int) (3)));
                final @Dimensionless TriangleEPA face1 = triangleStore.newTriangle(points, ((@Dimensionless int) (1)), ((@Dimensionless int) (2)), ((@Dimensionless int) (3)));
                final @Dimensionless TriangleEPA face2 = triangleStore.newTriangle(points, ((@Dimensionless int) (2)), ((@Dimensionless int) (0)), ((@Dimensionless int) (3)));
                final @Dimensionless TriangleEPA face3 = triangleStore.newTriangle(points, ((@Dimensionless int) (0)), ((@Dimensionless int) (2)), ((@Dimensionless int) (4)));
                final @Dimensionless TriangleEPA face4 = triangleStore.newTriangle(points, ((@Dimensionless int) (2)), ((@Dimensionless int) (1)), ((@Dimensionless int) (4)));
                final @Dimensionless TriangleEPA face5 = triangleStore.newTriangle(points, ((@Dimensionless int) (1)), ((@Dimensionless int) (0)), ((@Dimensionless int) (4)));
                if (!(face0 != null && face1 != null && face2 != null && face3 != null && face4 != null && face5 != null &&
                        face0.getDistSquare() > ((@Dimensionless int) (0)) && face1.getDistSquare() > ((@Dimensionless int) (0)) &&
                        face2.getDistSquare() > ((@Dimensionless int) (0)) && face3.getDistSquare() > ((@Dimensionless int) (0)) &&
                        face4.getDistSquare() > ((@Dimensionless int) (0)) && face5.getDistSquare() > ((@Dimensionless int) (0)))) {
                    return false;
                }
                TriangleEPA.link(new @Dimensionless EdgeEPA(face0, ((@Dimensionless int) (1))), new @Dimensionless EdgeEPA(face1, ((@Dimensionless int) (2))));
                TriangleEPA.link(new @Dimensionless EdgeEPA(face1, ((@Dimensionless int) (1))), new @Dimensionless EdgeEPA(face2, ((@Dimensionless int) (2))));
                TriangleEPA.link(new @Dimensionless EdgeEPA(face2, ((@Dimensionless int) (1))), new @Dimensionless EdgeEPA(face0, ((@Dimensionless int) (2))));
                TriangleEPA.link(new @Dimensionless EdgeEPA(face0, ((@Dimensionless int) (0))), new @Dimensionless EdgeEPA(face5, ((@Dimensionless int) (0))));
                TriangleEPA.link(new @Dimensionless EdgeEPA(face1, ((@Dimensionless int) (0))), new @Dimensionless EdgeEPA(face4, ((@Dimensionless int) (0))));
                TriangleEPA.link(new @Dimensionless EdgeEPA(face2, ((@Dimensionless int) (0))), new @Dimensionless EdgeEPA(face3, ((@Dimensionless int) (0))));
                TriangleEPA.link(new @Dimensionless EdgeEPA(face3, ((@Dimensionless int) (1))), new @Dimensionless EdgeEPA(face4, ((@Dimensionless int) (2))));
                TriangleEPA.link(new @Dimensionless EdgeEPA(face4, ((@Dimensionless int) (1))), new @Dimensionless EdgeEPA(face5, ((@Dimensionless int) (2))));
                TriangleEPA.link(new @Dimensionless EdgeEPA(face5, ((@Dimensionless int) (1))), new @Dimensionless EdgeEPA(face3, ((@Dimensionless int) (2))));
                nbTriangles = addFaceCandidate(face0, triangleHeap, nbTriangles, ((@Dimensionless float) (Float.MAX_VALUE)));
                nbTriangles = addFaceCandidate(face1, triangleHeap, nbTriangles, ((@Dimensionless float) (Float.MAX_VALUE)));
                nbTriangles = addFaceCandidate(face2, triangleHeap, nbTriangles, ((@Dimensionless float) (Float.MAX_VALUE)));
                nbTriangles = addFaceCandidate(face3, triangleHeap, nbTriangles, ((@Dimensionless float) (Float.MAX_VALUE)));
                nbTriangles = addFaceCandidate(face4, triangleHeap, nbTriangles, ((@Dimensionless float) (Float.MAX_VALUE)));
                nbTriangles = addFaceCandidate(face5, triangleHeap, nbTriangles, ((@Dimensionless float) (Float.MAX_VALUE)));
                nbVertices = ((@Dimensionless int) (5));
            }
            break;
        }
        if (nbTriangles == ((@Dimensionless int) (0))) {
            return false;
        }
        @Dimensionless
        TriangleEPA triangle;
        @Dimensionless
        float upperBoundSquarePenDepth = ((@Dimensionless float) (Float.MAX_VALUE));
        do {
            triangle = triangleHeap.remove();
            nbTriangles--;
            if (!triangle.isObsolete()) {
                if (nbVertices == MAX_SUPPORT_POINTS) {
                    break;
                }
                suppPointsA[nbVertices] = collisionShape1.getLocalSupportPointWithMargin(triangle.getClosestPoint());
                suppPointsB[nbVertices] = Transform.multiply(
                        body2Tobody1,
                        collisionShape2.getLocalSupportPointWithMargin(Matrix3x3.multiply(rotateToBody2, Vector3.negate(triangle.getClosestPoint()))));
                points[nbVertices] = Vector3.subtract(suppPointsA[nbVertices], suppPointsB[nbVertices]);
                final @Dimensionless int indexNewVertex = nbVertices;
                nbVertices++;
                final @Dimensionless float wDotv = points[indexNewVertex].dot(triangle.getClosestPoint());
                if (wDotv <= ((@Dimensionless int) (0))) {
                    throw new @Dimensionless IllegalStateException("wDotv must be greater than zero");
                }
                final @Dimensionless float wDotVSquare = wDotv * wDotv / triangle.getDistSquare();
                if (wDotVSquare < upperBoundSquarePenDepth) {
                    upperBoundSquarePenDepth = wDotVSquare;
                }
                final @Dimensionless float error = wDotv - triangle.getDistSquare();
                if (error <= Math.max(tolerance, GJKAlgorithm.REL_ERROR_SQUARE * wDotv)
                        || points[indexNewVertex].equals(points[triangle.get(((@Dimensionless int) (0)))])
                        || points[indexNewVertex].equals(points[triangle.get(((@Dimensionless int) (1)))])
                        || points[indexNewVertex].equals(points[triangle.get(((@Dimensionless int) (2)))])) {
                    break;
                }
                @Dimensionless
                int i = triangleStore.getNbTriangles();
                if (!triangle.computeSilhouette(points, indexNewVertex, triangleStore)) {
                    break;
                }
                while (i != triangleStore.getNbTriangles()) {
                    final @Dimensionless TriangleEPA newTriangle = triangleStore.get(i);
                    nbTriangles = addFaceCandidate(newTriangle, triangleHeap, nbTriangles, upperBoundSquarePenDepth);
                    i++;
                }
            }
        }
        while (nbTriangles > ((@Dimensionless int) (0)) && triangleHeap.element().getDistSquare() <= upperBoundSquarePenDepth);
        v.set(Matrix3x3.multiply(transform1.getOrientation().getMatrix(), triangle.getClosestPoint()));
        final @Dimensionless Vector3 pALocal = triangle.computeClosestPointOfObject(suppPointsA);
        final @Dimensionless Vector3 pBLocal = Transform.multiply(body2Tobody1.getInverse(), triangle.computeClosestPointOfObject(suppPointsB));
        final @Dimensionless Vector3 normal = v.getUnit();
        final @Dimensionless float penetrationDepth = v.length();
        if (penetrationDepth <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalStateException("penetration depth must be greater that zero");
        }
        contactInfo.set(normal, penetrationDepth, pALocal, pBLocal);
        return true;
    }

    // Decides if the origin is in the tetrahedron.
    // Returns 0 if the origin is in the tetrahedron or returns the index (1,2,3 or 4) of the bad
    // vertex if the origin is not in the tetrahedron.
    private static @Dimensionless int isOriginInTetrahedron(@Dimensionless Vector3 p1, @Dimensionless Vector3 p2, @Dimensionless Vector3 p3, @Dimensionless Vector3 p4) {
        final @Dimensionless Vector3 normal1 = Vector3.subtract(p2, p1).cross(Vector3.subtract(p3, p1));
        if (normal1.dot(p1) > ((@Dimensionless int) (0)) == normal1.dot(p4) > ((@Dimensionless int) (0))) {
            return ((@Dimensionless int) (4));
        }
        final @Dimensionless Vector3 normal2 = Vector3.subtract(p4, p2).cross(Vector3.subtract(p3, p2));
        if (normal2.dot(p2) > ((@Dimensionless int) (0)) == normal2.dot(p1) > ((@Dimensionless int) (0))) {
            return ((@Dimensionless int) (1));
        }
        final @Dimensionless Vector3 normal3 = Vector3.subtract(p4, p3).cross(Vector3.subtract(p1, p3));
        if (normal3.dot(p3) > ((@Dimensionless int) (0)) == normal3.dot(p2) > ((@Dimensionless int) (0))) {
            return ((@Dimensionless int) (2));
        }
        final @Dimensionless Vector3 normal4 = Vector3.subtract(p2, p4).cross(Vector3.subtract(p1, p4));
        if (normal4.dot(p4) > ((@Dimensionless int) (0)) == normal4.dot(p3) > ((@Dimensionless int) (0))) {
            return ((@Dimensionless int) (3));
        }
        return ((@Dimensionless int) (0));
    }

    // Adds a triangle face in the candidate triangle heap in the EPA algorithm.
    private static @Dimensionless int addFaceCandidate(@Dimensionless TriangleEPA triangle, @Dimensionless Queue<@Dimensionless TriangleEPA> heap, @Dimensionless int nbTriangles, @Dimensionless float upperBoundSquarePenDepth) {
        if (triangle.isClosestPointInternalToTriangle() && triangle.getDistSquare() <= upperBoundSquarePenDepth) {
            heap.add(triangle);
            nbTriangles++;
        }
        return nbTriangles;
    }

    // Compares the EPA triangles in the queue.
    @Dimensionless
    private static class TriangleComparison implements @Dimensionless Comparator<@Dimensionless TriangleEPA> {
        @Override
        public @Dimensionless int compare(EPAAlgorithm.@Dimensionless TriangleComparison this, @Dimensionless TriangleEPA face1, @Dimensionless TriangleEPA face2) {
            final @Dimensionless float dist1 = face1.getDistSquare();
            final @Dimensionless float dist2 = face2.getDistSquare();
            if (dist1 == dist2) {
                return ((@Dimensionless int) (0));
            }
            return dist1 > dist2 ? ((@Dimensionless int) (1)) : ((@Dimensionless int) (-1));
        }
    public TriangleComparison(EPAAlgorithm.@Dimensionless TriangleComparison this) { super(); }
    }
}
