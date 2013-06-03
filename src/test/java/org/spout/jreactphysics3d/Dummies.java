/*
 * This file is part of JReactPhysics3D.
 *
 * Copyright (c) 2013 Spout LLC <http://www.spout.org/>
 * JReactPhysics3D is licensed under the Spout License Version 1.
 *
 * JReactPhysics3D is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * In addition, 180 days after any changes are published, you can use the
 * software, incorporating those changes, under the terms of the MIT license,
 * as described in the Spout License Version 1.
 *
 * JReactPhysics3D is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for
 * more details.
 *
 * You should have received a copy of the GNU Lesser General Public License,
 * the MIT license and the Spout License Version 1 along with this program.
 * If not, see <http://www.gnu.org/licenses/> for the GNU Lesser General Public
 * License and see <http://spout.in/licensev1> for the full license, including
 * the MIT license.
 */
package org.spout.jreactphysics3d;

import java.util.Random;

import org.spout.jreactphysics3d.body.CollisionBody;
import org.spout.jreactphysics3d.collision.BroadPhasePair;
import org.spout.jreactphysics3d.collision.CollisionDetection;
import org.spout.jreactphysics3d.collision.ContactInfo;
import org.spout.jreactphysics3d.collision.shape.AABB;
import org.spout.jreactphysics3d.collision.shape.BoxShape;
import org.spout.jreactphysics3d.collision.shape.SphereShape;
import org.spout.jreactphysics3d.engine.CollisionWorld;
import org.spout.jreactphysics3d.mathematics.Quaternion;
import org.spout.jreactphysics3d.mathematics.Transform;
import org.spout.jreactphysics3d.mathematics.Vector3;

/**
 * Provides fake implementations and objects for tests.
 */
public class Dummies {
	private static final Random RANDOM = new Random();

	public static Vector3 newPosition() {
		return new Vector3(RANDOM.nextInt(21) - 10, RANDOM.nextInt(21) - 10, RANDOM.nextInt(21) - 10);
	}

	public static Quaternion newOrientation() {
		final float phi = RANDOM.nextFloat() * 2 * (float) Math.PI;
		final float theta = RANDOM.nextFloat() * 2 * (float) Math.PI;
		final float x = (float) Math.sin(theta) * (float) Math.cos(phi);
		final float y = (float) Math.sin(theta) * (float) Math.sin(phi);
		final float z = (float) Math.cos(theta);
		final float halfAngle = (float) Math.toRadians(RANDOM.nextFloat() * 2 * Math.PI) / 2;
		final float q = (float) Math.sin(halfAngle);
		return new Quaternion(x * q, y * q, z * q, (float) Math.cos(halfAngle));
	}

	public static Transform newTransform() {
		return new Transform(newPosition(), newOrientation());
	}

	public static CollisionBody newCollisionBody(int id) {
		return new CollisionBody(Transform.identity(), new BoxShape(new Vector3()), id);
	}

	public static AABB newAABB() {
		final Vector3 min = newPosition();
		return new AABB(min, Vector3.add(min, new Vector3(RANDOM.nextInt(3) + 2, RANDOM.nextInt(3) + 2, RANDOM.nextInt(3) + 2)));
	}

	public static AABB newIntersectingAABB(AABB with) {
		final Vector3 wMin = with.getMin();
		final Vector3 wSize = Vector3.subtract(with.getMax(), wMin);
		final int iSizeX = RANDOM.nextInt((int) wSize.getX() + 1);
		final int iSizeY = RANDOM.nextInt((int) wSize.getY() + 1);
		final int iSizeZ = RANDOM.nextInt((int) wSize.getZ() + 1);
		final int eSizeX = RANDOM.nextInt(3) + 2;
		final int eSizeY = RANDOM.nextInt(3) + 2;
		final int eSizeZ = RANDOM.nextInt(3) + 2;
		final Vector3 min = Vector3.subtract(wMin, new Vector3(eSizeX, eSizeY, eSizeZ));
		final Vector3 max = Vector3.add(wMin, new Vector3(iSizeX, iSizeY, iSizeZ));
		return new AABB(min, max);
	}

	public static SphereShape newSphereShape() {
		return new SphereShape(RANDOM.nextInt(3) + 2);
	}

	public static CollisionWorld newCollisionWorld() {
		return new DummyCollisionWorld();
	}

	public static CollisionDetection newCollisionDetection() {
		return new CollisionDetection(newCollisionWorld());
	}

	private static class DummyCollisionWorld extends CollisionWorld {
		private DummyCollisionWorld() {
		}

		@Override
		public void notifyAddedOverlappingPair(BroadPhasePair addedPair) {
		}

		@Override
		public void notifyRemovedOverlappingPair(BroadPhasePair removedPair) {
		}

		@Override
		public void notifyNewContact(BroadPhasePair pair, ContactInfo contactInfo) {
		}

		@Override
		public void updateOverlappingPair(BroadPhasePair pair) {
		}
	}
}
