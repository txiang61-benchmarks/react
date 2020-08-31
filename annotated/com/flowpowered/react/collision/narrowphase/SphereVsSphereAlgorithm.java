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
package com.flowpowered.react.collision.narrowphase;

import units.qual.Dimensionless;
import com.flowpowered.react.collision.shape.CollisionShape;
import com.flowpowered.react.collision.shape.SphereShape;
import com.flowpowered.react.constraint.ContactPoint.ContactPointInfo;
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector3;

/**
 * This class is used to compute the narrow-phase collision detection between two sphere shaped collision volumes.
 */
public class SphereVsSphereAlgorithm extends NarrowPhaseAlgorithm {
    @Override
    public @Dimensionless boolean testCollision(@Dimensionless SphereVsSphereAlgorithm this, @Dimensionless CollisionShape collisionShape1, @Dimensionless Transform transform1,
                                 @Dimensionless
                                 CollisionShape collisionShape2, @Dimensionless Transform transform2,
                                 @Dimensionless
                                 ContactPointInfo contactInfo) {
        final @Dimensionless SphereShape sphereShape1 = (@Dimensionless SphereShape) collisionShape1;
        final @Dimensionless SphereShape sphereShape2 = (@Dimensionless SphereShape) collisionShape2;
        final @Dimensionless Vector3 vectorBetweenCenters = Vector3.subtract(transform2.getPosition(), transform1.getPosition());
        final @Dimensionless float squaredDistanceBetweenCenters = vectorBetweenCenters.lengthSquare();
        final @Dimensionless float sumRadius = sphereShape1.getRadius() + sphereShape2.getRadius();
        if (squaredDistanceBetweenCenters <= sumRadius * sumRadius) {
            final @Dimensionless Vector3 centerSphere2InBody1LocalSpace = Transform.multiply(transform1.getInverse(), transform2.getPosition());
            final @Dimensionless Vector3 centerSphere1InBody2LocalSpace = Transform.multiply(transform2.getInverse(), transform1.getPosition());
            final @Dimensionless Vector3 intersectionOnBody1 = Vector3.multiply(sphereShape1.getRadius(), centerSphere2InBody1LocalSpace.getUnit());
            final @Dimensionless Vector3 intersectionOnBody2 = Vector3.multiply(sphereShape2.getRadius(), centerSphere1InBody2LocalSpace.getUnit());
            final @Dimensionless float penetrationDepth = sumRadius - (@Dimensionless float) Math.sqrt(squaredDistanceBetweenCenters);
            contactInfo.set(vectorBetweenCenters.getUnit(), penetrationDepth, intersectionOnBody1, intersectionOnBody2);
            return true;
        }
        return false;
    }
}
