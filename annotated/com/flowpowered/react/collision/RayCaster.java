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
package com.flowpowered.react.collision;

import units.qual.Dimensionless;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import com.flowpowered.react.body.CollisionBody;
import com.flowpowered.react.collision.shape.BoxShape;
import com.flowpowered.react.collision.shape.CapsuleShape;
import com.flowpowered.react.collision.shape.CollisionShape;
import com.flowpowered.react.collision.shape.ConeShape;
import com.flowpowered.react.collision.shape.CylinderShape;
import com.flowpowered.react.collision.shape.SphereShape;
import com.flowpowered.react.math.Matrix3x3;
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector3;

/**
 * Performs ray casting on collision shapes, finding the ones that intersect the ray.
 */
@Dimensionless
public class RayCaster {
    /**
     * Finds the closest of the bodies intersecting with the ray to the ray start. The ray is defined by a starting point and a direction. This method returns an {@link IntersectedBody} object
     * containing the body and the intersection point.
     *
     * @param rayStart The ray starting point
     * @param rayDir The ray direction
     * @param bodies The bodies to check for intersection
     * @return The closest body to the ray start and its intersection point
     */
    public static @Dimensionless IntersectedBody findClosestIntersectingBody(@Dimensionless Vector3 rayStart, @Dimensionless Vector3 rayDir, @Dimensionless Collection<@Dimensionless CollisionBody> bodies) {
        final @Dimensionless Map<@Dimensionless CollisionBody, @Dimensionless Vector3> intersecting = findIntersectingBodies(rayStart, rayDir, bodies);
        @Dimensionless
        CollisionBody closest = null;
        @Dimensionless
        float closestDistance = ((@Dimensionless float) (Float.MAX_VALUE));
        @Dimensionless
        Vector3 closestIntersectionPoint = null;
        for (@Dimensionless Entry<@Dimensionless CollisionBody, @Dimensionless Vector3> entry : intersecting.entrySet()) {
            final @Dimensionless Vector3 intersectionPoint = entry.getValue();
            final @Dimensionless float distance = Vector3.subtract(intersectionPoint, rayStart).lengthSquare();
            if (distance < closestDistance) {
                closest = entry.getKey();
                closestDistance = distance;
                closestIntersectionPoint = intersectionPoint;
            }
        }
        return closest != null ? new @Dimensionless IntersectedBody(closest, closestIntersectionPoint) : null;
    }

    /**
     * Finds the furthest of the bodies intersecting with the ray from the ray start. The ray is defined by a starting point and a direction. This method returns an {@link IntersectedBody} object
     * containing the body and the intersection point.
     *
     * @param rayStart The ray starting point
     * @param rayDir The ray direction
     * @param bodies The bodies to check for intersection
     * @return The furthest body from the ray start and its intersection point
     */
    public static @Dimensionless IntersectedBody findFurthestIntersectingBody(@Dimensionless Vector3 rayStart, @Dimensionless Vector3 rayDir, @Dimensionless Collection<@Dimensionless CollisionBody> bodies) {
        final @Dimensionless Map<@Dimensionless CollisionBody, @Dimensionless Vector3> intersecting = findIntersectingBodies(rayStart, rayDir, bodies);
        @Dimensionless
        CollisionBody furthest = null;
        @Dimensionless
        float furthestDistance = ((@Dimensionless float) (Float.MIN_VALUE));
        @Dimensionless
        Vector3 furthestIntersectionPoint = null;
        for (@Dimensionless Entry<@Dimensionless CollisionBody, @Dimensionless Vector3> entry : intersecting.entrySet()) {
            final @Dimensionless Vector3 intersectionPoint = entry.getValue();
            final @Dimensionless float distance = Vector3.subtract(intersectionPoint, rayStart).lengthSquare();
            if (distance > furthestDistance) {
                furthest = entry.getKey();
                furthestDistance = distance;
                furthestIntersectionPoint = intersectionPoint;
            }
        }
        return furthest != null ? new @Dimensionless IntersectedBody(furthest, furthestIntersectionPoint) : null;
    }

    /**
     * Finds all of the bodies intersecting with the ray. The ray is defined by a starting point and a direction. The bodies are returned mapped with the closest intersection point.
     *
     * @param rayStart The ray starting point
     * @param rayDir The ray direction
     * @param bodies The bodies to check for intersection
     * @return All of the intersection bodies, in no particular order, mapped to the intersection point
     */
    public static @Dimensionless Map<@Dimensionless CollisionBody, @Dimensionless Vector3> findIntersectingBodies(@Dimensionless Vector3 rayStart, @Dimensionless Vector3 rayDir, @Dimensionless Collection<@Dimensionless CollisionBody> bodies) {
        final @Dimensionless Map<@Dimensionless CollisionBody, @Dimensionless Vector3> intersecting = new @Dimensionless HashMap<>();
        final @Dimensionless Vector3 intersectionPoint = new @Dimensionless Vector3();
        for (@Dimensionless CollisionBody body : bodies) {
            if (intersects(rayStart, rayDir, body.getCollisionShape(), body.getTransform(), intersectionPoint)) {
                intersecting.put(body, new @Dimensionless Vector3(intersectionPoint));
            }
        }
        return intersecting;
    }

    // Tests for intersection between a ray defined by a starting point and a direction and a collision shape
    private static @Dimensionless boolean intersects(@Dimensionless Vector3 rayStart, @Dimensionless Vector3 rayDir, @Dimensionless CollisionShape shape, @Dimensionless Transform transform, @Dimensionless Vector3 intersectionPoint) {
        final @Dimensionless Transform worldToObject = transform.getInverse();
        final @Dimensionless Vector3 objRayStart = Transform.multiply(worldToObject, rayStart);
        final @Dimensionless Vector3 objRayDir = Matrix3x3.multiply(worldToObject.getOrientation().getMatrix(), rayDir);
        final @Dimensionless boolean intersects;
        switch (shape.getType()) {
            case BOX:
                intersects = intersects(objRayStart, objRayDir, (@Dimensionless BoxShape) shape, intersectionPoint);
                break;
            case SPHERE:
                intersects = intersects(objRayStart, objRayDir, (@Dimensionless SphereShape) shape, intersectionPoint);
                break;
            case CONE:
                intersects = intersects(objRayStart, objRayDir, (@Dimensionless ConeShape) shape, intersectionPoint);
                break;
            case CYLINDER:
                intersects = intersects(objRayStart, objRayDir, (@Dimensionless CylinderShape) shape, intersectionPoint);
                break;
            case CAPSULE:
                intersects = intersects(objRayStart, objRayDir, (@Dimensionless CapsuleShape) shape, intersectionPoint);
                break;
            case CONVEX_MESH:
                // TODO: implement this
                intersects = false;
                break;
            default:
                throw new @Dimensionless IllegalArgumentException("unknown collision shape");
        }
        if (intersects) {
            intersectionPoint.set(Transform.multiply(transform, intersectionPoint));
            return true;
        }
        return false;
    }

    // Tests for intersection between a ray defined by a starting point and a direction and a box
    private static @Dimensionless boolean intersects(@Dimensionless Vector3 rayStart, @Dimensionless Vector3 rayDir, @Dimensionless BoxShape box, @Dimensionless Vector3 intersectionPoint) {
        final @Dimensionless Vector3 extent = box.getExtent();
        final @Dimensionless Vector3 min = Vector3.negate(extent);
        final @Dimensionless Vector3 max = extent;
        @Dimensionless
        float t0;
        @Dimensionless
        float t1;
        if (rayDir.getX() >= ((@Dimensionless int) (0))) {
            t0 = (min.getX() - rayStart.getX()) / rayDir.getX();
            t1 = (max.getX() - rayStart.getX()) / rayDir.getX();
        } else {
            t0 = (max.getX() - rayStart.getX()) / rayDir.getX();
            t1 = (min.getX() - rayStart.getX()) / rayDir.getX();
        }
        final @Dimensionless float tyMin;
        final @Dimensionless float tyMax;
        if (rayDir.getY() >= ((@Dimensionless int) (0))) {
            tyMin = (min.getY() - rayStart.getY()) / rayDir.getY();
            tyMax = (max.getY() - rayStart.getY()) / rayDir.getY();
        } else {
            tyMin = (max.getY() - rayStart.getY()) / rayDir.getY();
            tyMax = (min.getY() - rayStart.getY()) / rayDir.getY();
        }
        if (t0 > tyMax || tyMin > t1) {
            return false;
        }
        if (tyMin > t0) {
            t0 = tyMin;
        }
        if (tyMax < t1) {
            t1 = tyMax;
        }
        final @Dimensionless float tzMin;
        final @Dimensionless float tzMax;
        if (rayDir.getZ() >= ((@Dimensionless int) (0))) {
            tzMin = (min.getZ() - rayStart.getZ()) / rayDir.getZ();
            tzMax = (max.getZ() - rayStart.getZ()) / rayDir.getZ();
        } else {
            tzMin = (max.getZ() - rayStart.getZ()) / rayDir.getZ();
            tzMax = (min.getZ() - rayStart.getZ()) / rayDir.getZ();
        }
        if (t0 > tzMax || tzMin > t1) {
            return false;
        }
        if (tzMin > t0) {
            t0 = tzMin;
        }
        if (tzMax < t1) {
            t1 = tzMax;
        }
        if (t1 >= ((@Dimensionless int) (0))) {
            final @Dimensionless float t;
            if (t0 >= ((@Dimensionless int) (0))) {
                t = t0;
            } else {
                t = t1;
            }
            intersectionPoint.set(Vector3.add(rayStart, Vector3.multiply(rayDir, t)));
            return true;
        }
        return false;
    }

    // Tests for intersection between a ray defined by a starting point and a direction and a sphere
    private static @Dimensionless boolean intersects(@Dimensionless Vector3 rayStart, @Dimensionless Vector3 rayDir, @Dimensionless SphereShape sphere, @Dimensionless Vector3 intersectionPoint) {
        final @Dimensionless float a = rayDir.dot(rayDir);
        final @Dimensionless float b = Vector3.multiply(rayDir, ((@Dimensionless int) (2))).dot(rayStart);
        final @Dimensionless float r = sphere.getRadius();
        final @Dimensionless float c = rayStart.dot(rayStart) - r * r;
        final @Dimensionless float discriminant = b * b - ((@Dimensionless int) (4)) * a * c;
        if (discriminant < ((@Dimensionless int) (0))) {
            return false;
        }
        final @Dimensionless float discriminantRoot = (@Dimensionless float) Math.sqrt(discriminant);
        @Dimensionless
        float t0 = (-b + discriminantRoot) / (((@Dimensionless int) (2)) * a);
        @Dimensionless
        float t1 = (-b - discriminantRoot) / (((@Dimensionless int) (2)) * a);
        if (t0 > t1) {
            final @Dimensionless float temp = t1;
            t1 = t0;
            t0 = temp;
        }
        if (t1 >= ((@Dimensionless int) (0))) {
            final @Dimensionless float t;
            if (t0 >= ((@Dimensionless int) (0))) {
                t = t0;
            } else {
                t = t1;
            }
            intersectionPoint.set(Vector3.add(rayStart, Vector3.multiply(rayDir, t)));
            return true;
        }
        return false;
    }

    // Tests for intersection between a ray defined by a starting point and a direction and a cone
    private static @Dimensionless boolean intersects(@Dimensionless Vector3 rayStart, @Dimensionless Vector3 rayDir, @Dimensionless ConeShape cone, @Dimensionless Vector3 intersectionPoint) {
        final @Dimensionless float vx = rayDir.getX();
        final @Dimensionless float vy = rayDir.getY();
        final @Dimensionless float vz = rayDir.getZ();
        final @Dimensionless float px = rayStart.getX();
        final @Dimensionless float py = rayStart.getY();
        final @Dimensionless float pz = rayStart.getZ();
        final @Dimensionless float r = cone.getRadius() / ((@Dimensionless int) (2));
        final @Dimensionless float h = cone.getHeight() / ((@Dimensionless int) (2));
        final @Dimensionless float r2 = r * r;
        final @Dimensionless float h2 = h * h;
        final @Dimensionless float a = vx * vx + vz * vz - (vy * vy * r2) / h2;
        final @Dimensionless float b = ((@Dimensionless int) (2)) * px * vx + ((@Dimensionless int) (2)) * pz * vz - (((@Dimensionless int) (2)) * r2 * py * vy) / h2 + (((@Dimensionless int) (2)) * r2 * vy) / h;
        final @Dimensionless float c = px * px + pz * pz - r * r - (r2 * py * py) / h2 + (((@Dimensionless int) (2)) * r2 * py) / h;
        final @Dimensionless float discriminant = b * b - ((@Dimensionless int) (4)) * a * c;
        @Dimensionless
        float tc0 = ((@Dimensionless float) (Float.MAX_VALUE));
        @Dimensionless
        float tc1 = ((@Dimensionless float) (Float.MAX_VALUE));
        if (discriminant >= ((@Dimensionless int) (0))) {
            final @Dimensionless float discriminantRoot = (@Dimensionless float) Math.sqrt(discriminant);
            final @Dimensionless float t0 = (-b + discriminantRoot) / (((@Dimensionless int) (2)) * a);
            if (t0 >= ((@Dimensionless int) (0))) {
                final @Dimensionless float py0 = py + vy * t0;
                if (py0 >= -h && py0 <= h) {
                    tc0 = t0;
                }
            }
            final @Dimensionless float t1 = (-b - discriminantRoot) / (((@Dimensionless int) (2)) * a);
            if (t1 >= ((@Dimensionless int) (0))) {
                final @Dimensionless float py1 = py + vy * t1;
                if (py1 >= -h && py1 <= h) {
                    tc1 = t1;
                }
            }
        }
        @Dimensionless
        float tb = ((@Dimensionless float) (Float.MAX_VALUE));
        if (vy != ((@Dimensionless int) (0))) {
            final @Dimensionless float t = (-h - py) / vy;
            if (t >= ((@Dimensionless int) (0))) {
                final @Dimensionless float rx = px + vx * t;
                final @Dimensionless float rz = pz + vz * t;
                if (rx * rx + rz * rz <= ((@Dimensionless int) (4)) * r * r) {
                    tb = t;
                }
            }
        }
        final @Dimensionless float t = tc0 < tc1 ? (tc0 < tb ? tc0 : tb) : (tc1 < tb ? tc1 : tb);
        if (t != ((@Dimensionless float) (Float.MAX_VALUE))) {
            intersectionPoint.set(Vector3.add(rayStart, Vector3.multiply(rayDir, t)));
            return true;
        }
        return false;
    }

    // Tests for intersection between a ray defined by a starting point and a direction and a cylinder
    private static @Dimensionless boolean intersects(@Dimensionless Vector3 rayStart, @Dimensionless Vector3 rayDir, @Dimensionless CylinderShape cylinder, @Dimensionless Vector3 intersectionPoint) {
        final @Dimensionless float vx = rayDir.getX();
        final @Dimensionless float vy = rayDir.getY();
        final @Dimensionless float vz = rayDir.getZ();
        final @Dimensionless float px = rayStart.getX();
        final @Dimensionless float py = rayStart.getY();
        final @Dimensionless float pz = rayStart.getZ();
        final @Dimensionless float r = cylinder.getRadius();
        final @Dimensionless float h = cylinder.getHeight() / ((@Dimensionless int) (2));
        final @Dimensionless float r2 = r * r;
        final @Dimensionless float a = vx * vx + vz * vz;
        final @Dimensionless float b = ((@Dimensionless int) (2)) * px * vx + ((@Dimensionless int) (2)) * pz * vz;
        final @Dimensionless float c = px * px + pz * pz - r2;
        final @Dimensionless float discriminant = b * b - ((@Dimensionless int) (4)) * a * c;
        @Dimensionless
        float tc0 = ((@Dimensionless float) (Float.MAX_VALUE));
        @Dimensionless
        float tc1 = ((@Dimensionless float) (Float.MAX_VALUE));
        if (discriminant >= ((@Dimensionless int) (0))) {
            final @Dimensionless float discriminantRoot = (@Dimensionless float) Math.sqrt(discriminant);
            final @Dimensionless float t0 = (-b + discriminantRoot) / (((@Dimensionless int) (2)) * a);
            if (t0 >= ((@Dimensionless int) (0))) {
                final @Dimensionless float ry0 = py + vy * t0;
                if (ry0 >= -h && ry0 <= h) {
                    tc0 = t0;
                }
            }
            final @Dimensionless float t1 = (-b - discriminantRoot) / (((@Dimensionless int) (2)) * a);
            if (t1 >= ((@Dimensionless int) (0))) {
                final @Dimensionless float ry1 = py + vy * t1;
                if (ry1 >= -h && ry1 <= h) {
                    tc1 = t1;
                }
            }
        }
        @Dimensionless
        float tb0 = ((@Dimensionless float) (Float.MAX_VALUE));
        @Dimensionless
        float tb1 = ((@Dimensionless float) (Float.MAX_VALUE));
        if (vy != ((@Dimensionless int) (0))) {
            final @Dimensionless float t0 = (h - py) / vy;
            if (t0 >= ((@Dimensionless int) (0))) {
                final @Dimensionless float rx = px + vx * t0;
                final @Dimensionless float rz = pz + vz * t0;
                if (rx * rx + rz * rz <= r2) {
                    tb0 = t0;
                }
            }
            final @Dimensionless float t1 = (-h - py) / vy;
            if (t1 >= ((@Dimensionless int) (0))) {
                final @Dimensionless float rx = px + vx * t1;
                final @Dimensionless float rz = pz + vz * t1;
                if (rx * rx + rz * rz <= r2) {
                    tb1 = t1;
                }
            }
        }
        @Dimensionless
        float t = tc0;
        if (tc1 < t) {
            t = tc1;
        }
        if (tb0 < t) {
            t = tb0;
        }
        if (tb1 < t) {
            t = tb1;
        }
        if (t != ((@Dimensionless float) (Float.MAX_VALUE))) {
            intersectionPoint.set(Vector3.add(rayStart, Vector3.multiply(rayDir, t)));
            return true;
        }
        return false;
    }

    // Tests for intersection between a ray defined by a starting point and a direction and a capsule
    private static @Dimensionless boolean intersects(@Dimensionless Vector3 rayStart, @Dimensionless Vector3 rayDir, @Dimensionless CapsuleShape capsule, @Dimensionless Vector3 intersectionPoint) {
        final @Dimensionless float vx = rayDir.getX();
        final @Dimensionless float vy = rayDir.getY();
        final @Dimensionless float vz = rayDir.getZ();
        final @Dimensionless float px = rayStart.getX();
        final @Dimensionless float py = rayStart.getY();
        final @Dimensionless float pz = rayStart.getZ();
        final @Dimensionless float r = capsule.getRadius();
        final @Dimensionless float h = capsule.getHeight() / ((@Dimensionless int) (2));
        final @Dimensionless float r2 = r * r;
        final @Dimensionless float a = vx * vx + vz * vz;
        final @Dimensionless float b = ((@Dimensionless int) (2)) * px * vx + ((@Dimensionless int) (2)) * pz * vz;
        final @Dimensionless float c = px * px + pz * pz - r2;
        final @Dimensionless float discriminant = b * b - ((@Dimensionless int) (4)) * a * c;
        @Dimensionless
        float tc0 = ((@Dimensionless float) (Float.MAX_VALUE));
        @Dimensionless
        float tc1 = ((@Dimensionless float) (Float.MAX_VALUE));
        if (discriminant >= ((@Dimensionless int) (0))) {
            final @Dimensionless float discriminantRoot = (@Dimensionless float) Math.sqrt(discriminant);
            final @Dimensionless float t0 = (-b + discriminantRoot) / (((@Dimensionless int) (2)) * a);
            if (t0 >= ((@Dimensionless int) (0))) {
                final @Dimensionless float ry0 = py + vy * t0;
                if (ry0 >= -h && ry0 <= h) {
                    tc0 = t0;
                }
            }
            final @Dimensionless float t1 = (-b - discriminantRoot) / (((@Dimensionless int) (2)) * a);
            if (t1 >= ((@Dimensionless int) (0))) {
                final @Dimensionless float ry1 = py + vy * t1;
                if (ry1 >= -h && ry1 <= h) {
                    tc1 = t1;
                }
            }
        }
        final @Dimensionless float t = Math.min(tc0, tc1);
        if (t != ((@Dimensionless float) (Float.MAX_VALUE))) {
            intersectionPoint.set(Vector3.add(rayStart, Vector3.multiply(rayDir, t)));
            return true;
        }
        final @Dimensionless SphereShape sphere = new @Dimensionless SphereShape(capsule.getRadius());
        final @Dimensionless Vector3 dy = new @Dimensionless Vector3(((@Dimensionless int) (0)), h, ((@Dimensionless int) (0)));
        final @Dimensionless Vector3 rayStartTop = Vector3.subtract(rayStart, dy);
        final @Dimensionless Vector3 rayStartBottom = Vector3.add(rayStart, dy);
        return intersects(rayStartTop, rayDir, sphere, intersectionPoint) || intersects(rayStartBottom, rayDir, sphere, intersectionPoint);
    }

    /**
     * Represents a body that was intersected by a ray. This class stores the body and the intersection point.
     */
    @Dimensionless
    public static class IntersectedBody {
        private final @Dimensionless CollisionBody body;
        private final @Dimensionless Vector3 intersectionPoint;

        private IntersectedBody(RayCaster.@Dimensionless IntersectedBody this, @Dimensionless CollisionBody body, @Dimensionless Vector3 intersectionPoint) {
            this.body = body;
            this.intersectionPoint = intersectionPoint;
        }

        /**
         * Gets the intersected body.
         *
         * @return The body
         */
        public @Dimensionless CollisionBody getBody(RayCaster.@Dimensionless IntersectedBody this) {
            return body;
        }

        /**
         * Gets the intersection point in world space.
         *
         * @return The intersection point
         */
        public @Dimensionless Vector3 getIntersectionPoint(RayCaster.@Dimensionless IntersectedBody this) {
            return intersectionPoint;
        }
    }
}
