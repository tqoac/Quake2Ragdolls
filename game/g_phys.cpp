/*
Copyright (C) 1997-2001 Id Software, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  

See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

*/
// g_phys.c

#include "g_local.h"

#include "btBulletDynamicsCommon.h"
#include <Windows.h>

#include <vector>

class testQuery : public btBroadphaseAabbCallback 
{
public:
	std::vector<std::pair<btRigidBody*, float>> bodies;
	vec3_t center;

	bool	process(const btBroadphaseProxy* proxy)
	{
		btCollisionObject *coll = (btCollisionObject*)proxy->m_clientObject;

		if (!coll->isStaticOrKinematicObject())
		{
			vec3_t dist;
			Vec3Copy (coll->getWorldTransform().getOrigin(), dist);

			vec3_t st, en;
			Vec3Copy(coll->getWorldTransform().getOrigin(), en);
			Vec3Copy(center, st);

			cmTrace_t tr = gi.trace(st, vec3Origin, vec3Origin, en, NULL, CONTENTS_SOLID);

			if (tr.fraction == 1.0)
			{
				Vec3Subtract(dist, center, dist);

				bodies.push_back(std::pair<btRigidBody*, float>((btRigidBody*)coll, Vec3Length(dist)));
			}
		}

		return false;
	}
};

/*


pushmove objects do not obey gravity, and do not interact with each other or trigger fields, but block normal movement and push normal objects when they move.

onground is set for toss objects when they come to a complete rest.  it is set for steping or walking objects 

doors, plats, etc are SOLID_BSP, and MOVETYPE_PUSH
bonus items are SOLID_TRIGGER touch, and MOVETYPE_TOSS
corpses are SOLID_NOT and MOVETYPE_TOSS
crates are SOLID_BBOX and MOVETYPE_TOSS
walking monsters are SOLID_SLIDEBOX and MOVETYPE_STEP
flying/floating monsters are SOLID_SLIDEBOX and MOVETYPE_FLY

solid_edge items only clip against bsp models.

*/


/*
============
SV_TestEntityPosition

============
*/
edict_t	*SV_TestEntityPosition (edict_t *ent)
{
	cmTrace_t	trace;
	int			mask;

	if (ent->clipMask)
		mask = ent->clipMask;
	else
		mask = CONTENTS_MASK_SOLID;
	trace = gi.trace (ent->s.origin, ent->mins, ent->maxs, ent->s.origin, ent, mask);

	if (trace.startSolid)
		return g_edicts;

	return NULL;
}


/*
================
SV_CheckVelocity
================
*/
void SV_CheckVelocity (edict_t *ent)
{
	int		i;

	//
	// bound velocity
	//
	for (i=0 ; i<3 ; i++)
	{
		if (ent->velocity[i] > sv_maxvelocity->floatVal)
			ent->velocity[i] = sv_maxvelocity->floatVal;
		else if (ent->velocity[i] < -sv_maxvelocity->floatVal)
			ent->velocity[i] = -sv_maxvelocity->floatVal;
	}
}

/*
=============
SV_RunThink

Runs thinking code for this frame if necessary
=============
*/
bool SV_RunThink (edict_t *ent)
{
	float	thinktime;

	thinktime = ent->nextthink;
	if (thinktime <= 0)
		return true;
	if (thinktime > level.time+0.001)
		return true;

	ent->nextthink = 0;
	if (!ent->think)
		gi.error ("NULL ent->think");
	ent->think (ent);

	return false;
}

/*
==================
SV_Impact

Two entities have touched, so run their touch functions
==================
*/
void SV_Impact (edict_t *e1, cmTrace_t *trace)
{
	edict_t		*e2;
	//	plane_t	backplane;

	e2 = trace->ent;

	if (e1->touch && e1->solid != SOLID_NOT)
		e1->touch (e1, e2, &trace->plane, trace->surface);

	if (e2->touch && e2->solid != SOLID_NOT)
		e2->touch (e2, e1, NULL, NULL);
}


/*
==================
ClipVelocity

Slide off of the impacting object
returns the blocked flags (1 = floor, 2 = step / wall)
==================
*/
#define STOP_EPSILON	0.1

int ClipVelocity (vec3_t in, vec3_t normal, vec3_t out, float overbounce)
{
	float	backoff;
	float	change;
	int		i, blocked;

	blocked = 0;
	if (normal[2] > 0)
		blocked |= 1;		// floor
	if (!normal[2])
		blocked |= 2;		// step

	backoff = DotProduct (in, normal) * overbounce;

	for (i=0 ; i<3 ; i++)
	{
		change = normal[i]*backoff;
		out[i] = in[i] - change;
		if (out[i] > -STOP_EPSILON && out[i] < STOP_EPSILON)
			out[i] = 0;
	}

	return blocked;
}


/*
============
SV_FlyMove

The basic solid body movement clip that slides along multiple planes
Returns the clipflags if the velocity was modified (hit something solid)
1 = floor
2 = wall / step
4 = dead stop
============
*/
#define MAX_CLIP_PLANES	5
int SV_FlyMove (edict_t *ent, float time, int mask)
{
	edict_t		*hit;
	int			bumpcount, numbumps;
	vec3_t		dir;
	float		d;
	int			numplanes;
	vec3_t		planes[MAX_CLIP_PLANES];
	vec3_t		primal_velocity, original_velocity, new_velocity;
	int			i, j;
	cmTrace_t	trace;
	vec3_t		end;
	float		time_left;
	int			blocked;

	numbumps = 4;

	blocked = 0;
	Vec3Copy (ent->velocity, original_velocity);
	Vec3Copy (ent->velocity, primal_velocity);
	numplanes = 0;

	time_left = time;

	ent->groundentity = NULL;
	for (bumpcount=0 ; bumpcount<numbumps ; bumpcount++)
	{
		for (i=0 ; i<3 ; i++)
			end[i] = ent->s.origin[i] + time_left * ent->velocity[i];

		trace = gi.trace (ent->s.origin, ent->mins, ent->maxs, end, ent, mask);

		if (trace.allSolid)
		{	// entity is trapped in another solid
			Vec3Copy (vec3Origin, ent->velocity);
			return 3;
		}

		if (trace.fraction > 0)
		{	// actually covered some distance
			Vec3Copy (trace.endPos, ent->s.origin);
			Vec3Copy (ent->velocity, original_velocity);
			numplanes = 0;
		}

		if (trace.fraction == 1)
			break;		// moved the entire distance

		hit = trace.ent;

		if (trace.plane.normal[2] > 0.7)
		{
			blocked |= 1;		// floor
			if ( hit->solid == SOLID_BSP)
			{
				ent->groundentity = hit;
				ent->groundentity_linkcount = hit->linkCount;
			}
		}
		if (!trace.plane.normal[2])
		{
			blocked |= 2;		// step
		}

		//
		// run the impact function
		//
		SV_Impact (ent, &trace);
		if (!ent->inUse)
			break;		// removed by the impact function


		time_left -= time_left * trace.fraction;

		// cliped to another plane
		if (numplanes >= MAX_CLIP_PLANES)
		{	// this shouldn't really happen
			Vec3Copy (vec3Origin, ent->velocity);
			return 3;
		}

		Vec3Copy (trace.plane.normal, planes[numplanes]);
		numplanes++;

		//
		// modify original_velocity so it parallels all of the clip planes
		//
		for (i=0 ; i<numplanes ; i++)
		{
			ClipVelocity (original_velocity, planes[i], new_velocity, 1);

			for (j=0 ; j<numplanes ; j++)
				if ((j != i) && !Vec3Compare (planes[i], planes[j]))
				{
					if (DotProduct (new_velocity, planes[j]) < 0)
						break;	// not ok
				}
				if (j == numplanes)
					break;
		}

		if (i != numplanes)
		{	// go along this plane
			Vec3Copy (new_velocity, ent->velocity);
		}
		else
		{	// go along the crease
			if (numplanes != 2)
			{
				//				gi.dprintf ("clip velocity, numplanes == %i\n",numplanes);
				Vec3Copy (vec3Origin, ent->velocity);
				return 7;
			}
			CrossProduct (planes[0], planes[1], dir);
			d = DotProduct (dir, ent->velocity);
			Vec3Scale (dir, d, ent->velocity);
		}

		//
		// if original velocity is against the original velocity, stop dead
		// to avoid tiny occilations in sloping corners
		//
		if (DotProduct (ent->velocity, primal_velocity) <= 0)
		{
			Vec3Copy (vec3Origin, ent->velocity);
			return blocked;
		}
	}

	return blocked;
}


/*
============
SV_AddGravity

============
*/
void SV_AddGravity (edict_t *ent)
{
	ent->velocity[2] -= ent->gravity * sv_gravity->floatVal * FRAMETIME;
}

/*
===============================================================================

PUSHMOVE

===============================================================================
*/

/*
============
SV_PushEntity

Does not change the entities velocity at all
============
*/
cmTrace_t SV_PushEntity (edict_t *ent, vec3_t push)
{
	cmTrace_t	trace;
	vec3_t		start;
	vec3_t		end;
	int			mask;

	Vec3Copy (ent->s.origin, start);
	Vec3Add (start, push, end);

retry:
	if (ent->clipMask)
		mask = ent->clipMask;
	else
		mask = CONTENTS_MASK_SOLID;

	trace = gi.trace (start, ent->mins, ent->maxs, end, ent, mask);

	Vec3Copy (trace.endPos, ent->s.origin);
	gi.linkentity (ent);

	if (trace.fraction != 1.0)
	{
		SV_Impact (ent, &trace);

		// if the pushed entity went away and the pusher is still there
		if (!trace.ent->inUse && ent->inUse)
		{
			// move the pusher back and try again
			Vec3Copy (start, ent->s.origin);
			gi.linkentity (ent);
			goto retry;
		}
	}

	if (ent->inUse)
		G_TouchTriggers (ent);

	return trace;
}					


struct pushed_t
{
	edict_t	*ent;
	vec3_t	origin;
	vec3_t	angles;
	float	deltayaw;
};
pushed_t	pushed[MAX_CS_EDICTS], *pushed_p;

edict_t	*obstacle;

/*
============
SV_Push

Objects need to be moved back on a failed push,
otherwise riders would continue to slide.
============
*/
bool SV_Push (edict_t *pusher, vec3_t move, vec3_t amove)
{
	int			i, e;
	edict_t		*check, *block;
	vec3_t		mins, maxs;
	pushed_t	*p;
	vec3_t		org, org2, move2, forward, right, up;

	// clamp the move to 1/8 units, so the position will
	// be accurate for client side prediction
	for (i=0 ; i<3 ; i++)
	{
		float	temp;
		temp = move[i]*8.0;
		if (temp > 0.0)
			temp += 0.5;
		else
			temp -= 0.5;
		move[i] = 0.125 * (int)temp;
	}

	// find the bounding box
	for (i=0 ; i<3 ; i++)
	{
		mins[i] = pusher->absMin[i] + move[i];
		maxs[i] = pusher->absMax[i] + move[i];
	}

	// we need this for pushing things later
	Vec3Subtract (vec3Origin, amove, org);
	Angles_Vectors (org, forward, right, up);

	// save the pusher's original position
	pushed_p->ent = pusher;
	Vec3Copy (pusher->s.origin, pushed_p->origin);
	Vec3Copy (pusher->s.angles, pushed_p->angles);
	if (pusher->client)
		pushed_p->deltayaw = pusher->client->ps.pMove.deltaAngles[YAW];
	pushed_p++;

	// move the pusher to it's final position
	Vec3Add (pusher->s.origin, move, pusher->s.origin);
	Vec3Add (pusher->s.angles, amove, pusher->s.angles);
	gi.linkentity (pusher);

	// see if any solid entities are inside the final position
	check = g_edicts+1;
	for (e = 1; e < globals.numEdicts; e++, check++)
	{
		if (!check->inUse)
			continue;
		if (check->movetype == MOVETYPE_PUSH
			|| check->movetype == MOVETYPE_STOP
			|| check->movetype == MOVETYPE_NONE
			|| check->movetype == MOVETYPE_NOCLIP)
			continue;

		if (!check->area.prev)
			continue;		// not linked in anywhere

		// if the entity is standing on the pusher, it will definitely be moved
		if (check->groundentity != pusher)
		{
			// see if the ent needs to be tested
			if (check->absMin[0] >= maxs[0]
			|| check->absMin[1] >= maxs[1]
			|| check->absMin[2] >= maxs[2]
			|| check->absMax[0] <= mins[0]
			|| check->absMax[1] <= mins[1]
			|| check->absMax[2] <= mins[2])
				continue;

			// see if the ent's bbox is inside the pusher's final position
			if (!SV_TestEntityPosition (check))
				continue;
		}

		if ((pusher->movetype == MOVETYPE_PUSH) || (check->groundentity == pusher))
		{
			// move this entity
			pushed_p->ent = check;
			Vec3Copy (check->s.origin, pushed_p->origin);
			Vec3Copy (check->s.angles, pushed_p->angles);
			pushed_p++;

			// try moving the contacted entity 
			Vec3Add (check->s.origin, move, check->s.origin);
			if (check->client)
			{	// FIXME: doesn't rotate monsters?
				check->client->ps.pMove.deltaAngles[YAW] += amove[YAW];
			}

			// figure movement due to the pusher's amove
			Vec3Subtract (check->s.origin, pusher->s.origin, org);
			org2[0] = DotProduct (org, forward);
			org2[1] = -DotProduct (org, right);
			org2[2] = DotProduct (org, up);
			Vec3Subtract (org2, org, move2);
			Vec3Add (check->s.origin, move2, check->s.origin);

			// may have pushed them off an edge
			if (check->groundentity != pusher)
				check->groundentity = NULL;

			block = SV_TestEntityPosition (check);
			if (!block)
			{	// pushed ok
				gi.linkentity (check);
				// impact?
				continue;
			}

			// if it is ok to leave in the old position, do it
			// this is only relevent for riding entities, not pushed
			// FIXME: this doesn't acount for rotation
			Vec3Subtract (check->s.origin, move, check->s.origin);
			block = SV_TestEntityPosition (check);
			if (!block)
			{
				pushed_p--;
				continue;
			}
		}

		// save off the obstacle so we can call the block function
		obstacle = check;

		// move back any entities we already moved
		// go backwards, so if the same entity was pushed
		// twice, it goes back to the original position
		for (p=pushed_p-1 ; p>=pushed ; p--)
		{
			Vec3Copy (p->origin, p->ent->s.origin);
			Vec3Copy (p->angles, p->ent->s.angles);
			if (p->ent->client)
			{
				p->ent->client->ps.pMove.deltaAngles[YAW] = p->deltayaw;
			}
			gi.linkentity (p->ent);
		}
		return false;
	}

	//FIXME: is there a better way to handle this?
	// see if anything we moved has touched a trigger
	for (p=pushed_p-1 ; p>=pushed ; p--)
		G_TouchTriggers (p->ent);

	return true;
}

/*
================
SV_Physics_Pusher

Bmodel objects don't interact with each other, but
push all box objects
================
*/
void SV_Physics_Pusher (edict_t *ent)
{
	vec3_t		move, amove;
	edict_t		*part, *mv;

	// if not a team captain, so movement will be handled elsewhere
	if ( ent->flags & FL_TEAMSLAVE)
		return;

	// make sure all team slaves can move before commiting
	// any moves or calling any think functions
	// if the move is blocked, all moved objects will be backed out
	//retry:
	pushed_p = pushed;
	for (part = ent ; part ; part=part->teamchain)
	{
		if (part->velocity[0] || part->velocity[1] || part->velocity[2] ||
			part->avelocity[0] || part->avelocity[1] || part->avelocity[2]
		)
		{	// object is moving
			Vec3Scale (part->velocity, FRAMETIME, move);
			Vec3Scale (part->avelocity, FRAMETIME, amove);

			if (!SV_Push (part, move, amove))
				break;	// move was blocked
		}
	}
	if (pushed_p > &pushed[MAX_CS_EDICTS])
		gi.error ("pushed_p > &pushed[MAX_CS_EDICTS], memory corrupted");

	if (part)
	{
		// the move failed, bump all nextthink times and back out moves
		for (mv = ent ; mv ; mv=mv->teamchain)
		{
			if (mv->nextthink > 0)
				mv->nextthink += FRAMETIME;
		}

		// if the pusher has a "blocked" function, call it
		// otherwise, just stay in place until the obstacle is gone
		if (part->blocked)
			part->blocked (part, obstacle);
#if 0
		// if the pushed entity went away and the pusher is still there
		if (!obstacle->inUse && part->inUse)
			goto retry;
#endif
	}
	else
	{
		// the move succeeded, so call all think functions
		for (part = ent ; part ; part=part->teamchain)
		{
			SV_RunThink (part);
		}
	}
}

//==================================================================

/*
=============
SV_Physics_None

Non moving objects can only think
=============
*/
void SV_Physics_None (edict_t *ent)
{
	// regular thinking
	SV_RunThink (ent);
}

/*
=============
SV_Physics_Noclip

A moving object that doesn't obey physics
=============
*/
void SV_Physics_Noclip (edict_t *ent)
{
	// regular thinking
	if (!SV_RunThink (ent))
		return;

	Vec3MA (ent->s.angles, FRAMETIME, ent->avelocity, ent->s.angles);
	Vec3MA (ent->s.origin, FRAMETIME, ent->velocity, ent->s.origin);

	gi.linkentity (ent);
}

/*
==============================================================================

TOSS / BOUNCE

==============================================================================
*/

/*
=============
SV_Physics_Toss

Toss, bounce, and fly movement.  When onground, do nothing.
=============
*/
void SV_Physics_Toss (edict_t *ent)
{
	cmTrace_t	trace;
	vec3_t		move;
	float		backoff;
	edict_t		*slave;
	bool		wasinwater;
	bool		isinwater;
	vec3_t		old_origin;

	// regular thinking
	SV_RunThink (ent);

	// if not a team captain, so movement will be handled elsewhere
	if ( ent->flags & FL_TEAMSLAVE)
		return;

	if (ent->velocity[2] > 0)
		ent->groundentity = NULL;

	// check for the groundentity going away
	if (ent->groundentity)
		if (!ent->groundentity->inUse)
			ent->groundentity = NULL;

	// if onground, return without moving
	if ( ent->groundentity )
		return;

	Vec3Copy (ent->s.origin, old_origin);

	SV_CheckVelocity (ent);

	// add gravity
	if (ent->movetype != MOVETYPE_FLY
		&& ent->movetype != MOVETYPE_FLYMISSILE)
		SV_AddGravity (ent);

	// move angles
	Vec3MA (ent->s.angles, FRAMETIME, ent->avelocity, ent->s.angles);

	// move origin
	Vec3Scale (ent->velocity, FRAMETIME, move);
	trace = SV_PushEntity (ent, move);
	if (!ent->inUse)
		return;

	if (trace.fraction < 1)
	{
		if (ent->movetype == MOVETYPE_BOUNCE)
			backoff = 1.5;
		else
			backoff = 1;

		ClipVelocity (ent->velocity, trace.plane.normal, ent->velocity, backoff);

		// stop if on ground
		if (trace.plane.normal[2] > 0.7)
		{		
			if (ent->velocity[2] < 60 || ent->movetype != MOVETYPE_BOUNCE )
			{
				ent->groundentity = trace.ent;
				ent->groundentity_linkcount = trace.ent->linkCount;
				Vec3Copy (vec3Origin, ent->velocity);
				Vec3Copy (vec3Origin, ent->avelocity);
			}
		}

		//		if (ent->touch)
		//			ent->touch (ent, trace.ent, &trace.plane, trace.surface);
	}

	// check for water transition
	wasinwater = (ent->watertype & CONTENTS_MASK_WATER) ? true : false;
	ent->watertype = gi.pointcontents (ent->s.origin);
	isinwater = (ent->watertype & CONTENTS_MASK_WATER) ? true : false;

	if (isinwater)
		ent->waterlevel = 1;
	else
		ent->waterlevel = 0;

	if (!wasinwater && isinwater)
		gi.positioned_sound (old_origin, g_edicts, CHAN_AUTO, gi.soundindex("misc/h2ohit1.wav"), 1, 1, 0);
	else if (wasinwater && !isinwater)
		gi.positioned_sound (ent->s.origin, g_edicts, CHAN_AUTO, gi.soundindex("misc/h2ohit1.wav"), 1, 1, 0);

	// move teamslaves
	for (slave = ent->teamchain; slave; slave = slave->teamchain)
	{
		Vec3Copy (ent->s.origin, slave->s.origin);
		gi.linkentity (slave);
	}
}

/*
===============================================================================

STEPPING MOVEMENT

===============================================================================
*/

/*
=============
SV_Physics_Step

Monsters freefall when they don't have a ground entity, otherwise
all movement is done with discrete steps.

This is also used for objects that have become still on the ground, but
will fall if the floor is pulled out from under them.
FIXME: is this true?
=============
*/

//FIXME: hacked in for E3 demo
#define sv_stopspeed		100
#define sv_friction			6
#define sv_waterfriction	1

void SV_AddRotationalFriction (edict_t *ent)
{
	int		n;
	float	adjustment;

	Vec3MA (ent->s.angles, FRAMETIME, ent->avelocity, ent->s.angles);
	adjustment = FRAMETIME * sv_stopspeed * sv_friction;
	for (n = 0; n < 3; n++)
	{
		if (ent->avelocity[n] > 0)
		{
			ent->avelocity[n] -= adjustment;
			if (ent->avelocity[n] < 0)
				ent->avelocity[n] = 0;
		}
		else
		{
			ent->avelocity[n] += adjustment;
			if (ent->avelocity[n] > 0)
				ent->avelocity[n] = 0;
		}
	}
}

void SV_Physics_Step (edict_t *ent)
{
	bool	wasonground;
	bool	hitsound = false;
	float		*vel;
	float		speed, newspeed, control;
	float		friction;
	edict_t		*groundentity;
	int			mask;

	// airborn monsters should always check for ground
	if (!ent->groundentity)
		M_CheckGround (ent);

	groundentity = ent->groundentity;

	SV_CheckVelocity (ent);

	if (groundentity)
		wasonground = true;
	else
		wasonground = false;

	if (ent->avelocity[0] || ent->avelocity[1] || ent->avelocity[2])
		SV_AddRotationalFriction (ent);

	// add gravity except:
	//   flying monsters
	//   swimming monsters who are in the water
	if (! wasonground)
		if (!(ent->flags & FL_FLY))
			if (!((ent->flags & FL_SWIM) && (ent->waterlevel > 2)))
			{
				if (ent->velocity[2] < sv_gravity->floatVal*-0.1)
					hitsound = true;
				if (ent->waterlevel == 0)
					SV_AddGravity (ent);
			}

			// friction for flying monsters that have been given vertical velocity
			if ((ent->flags & FL_FLY) && (ent->velocity[2] != 0))
			{
				speed = fabs(ent->velocity[2]);
				control = speed < sv_stopspeed ? sv_stopspeed : speed;
				friction = sv_friction/3;
				newspeed = speed - (FRAMETIME * control * friction);
				if (newspeed < 0)
					newspeed = 0;
				newspeed /= speed;
				ent->velocity[2] *= newspeed;
			}

			// friction for flying monsters that have been given vertical velocity
			if ((ent->flags & FL_SWIM) && (ent->velocity[2] != 0))
			{
				speed = fabs(ent->velocity[2]);
				control = speed < sv_stopspeed ? sv_stopspeed : speed;
				newspeed = speed - (FRAMETIME * control * sv_waterfriction * ent->waterlevel);
				if (newspeed < 0)
					newspeed = 0;
				newspeed /= speed;
				ent->velocity[2] *= newspeed;
			}

			if (ent->velocity[2] || ent->velocity[1] || ent->velocity[0])
			{
				// apply friction
				// let dead monsters who aren't completely onground slide
				if ((wasonground) || (ent->flags & (FL_SWIM|FL_FLY)))
					if (!(ent->health <= 0.0 && !M_CheckBottom(ent)))
					{
						vel = ent->velocity;
						speed = sqrtf(vel[0]*vel[0] +vel[1]*vel[1]);
						if (speed)
						{
							friction = sv_friction;

							control = speed < sv_stopspeed ? sv_stopspeed : speed;
							newspeed = speed - FRAMETIME*control*friction;

							if (newspeed < 0)
								newspeed = 0;
							newspeed /= speed;

							vel[0] *= newspeed;
							vel[1] *= newspeed;
						}
					}

					if (ent->svFlags & SVF_MONSTER)
						mask = CONTENTS_MASK_MONSTERSOLID;
					else
						mask = CONTENTS_MASK_SOLID;
					SV_FlyMove (ent, FRAMETIME, mask);

					gi.linkentity (ent);
					G_TouchTriggers (ent);
					if (!ent->inUse)
						return;

					if (ent->groundentity)
						if (!wasonground)
							if (hitsound)
								gi.sound (ent, 0, gi.soundindex("world/land.wav"), 1, 1, 0);
			}

			// regular thinking
			SV_RunThink (ent);
}

//============================================================================
/*
================
G_RunEntity

================
*/
void G_RunEntity (edict_t *ent)
{
	if (ent->prethink)
		ent->prethink (ent);

	switch ( (int)ent->movetype)
	{
	case MOVETYPE_PUSH:
	case MOVETYPE_STOP:
		SV_Physics_Pusher (ent);
		break;
	case MOVETYPE_NONE:
		SV_Physics_None (ent);
		break;
	case MOVETYPE_NOCLIP:
		SV_Physics_Noclip (ent);
		break;
	case MOVETYPE_STEP:
		SV_Physics_Step (ent);
		break;
	case MOVETYPE_TOSS:
	case MOVETYPE_BOUNCE:
	case MOVETYPE_FLY:
	case MOVETYPE_FLYMISSILE:
		SV_Physics_Toss (ent);
		break;
	default:
		gi.error ("SV_Physics: bad movetype %i", (int)ent->movetype);			
	}
}


static btDiscreteDynamicsWorld *physicsWorld;
static btDefaultCollisionConfiguration *physicsConfig;
static btCollisionDispatcher *physicsDispatcher;
static btAxisSweep3 *physicsBroadphase;
static btSequentialImpulseConstraintSolver *physicsSolver;

class myRigidBody : public btRigidBody
{
public:
	myRigidBody (const btRigidBodyConstructionInfo& info) :
	  btRigidBody(info)
	  {
		  normalAngularDamping = normalLinearDamping = 0;
	  }
	  float normalAngularDamping;
	  float normalLinearDamping;

public:
	void setNormalDamping (float lin, float ang)
	{
		normalAngularDamping = ang;
		normalLinearDamping = lin;

		setDamping(lin, ang);
	}

	float getNormalAngularDamping ()
	{
		return normalAngularDamping;
	}

	float getNormalLinearDamping ()
	{
		return normalLinearDamping;
	}
};

btQuaternion ConvertQuat (btQuaternion &quat)
{
	btMatrix3x3 mat (quat);
	btMatrix3x3 tempMat;

	for (int x = 0; x < 3; ++x)
		for (int y = 0; y < 3; ++y)
		{
			tempMat[x][y] = mat[y][x];
		}

		btQuaternion q;
		tempMat.getRotation(q);
		return q;
}
class QuakeBodyMotionState : public btMotionState
{
public:
	QuakeBodyMotionState(const btTransform &initialpos, edict_t *entity)
	{
		canReset = false;
		resetPosition.setIdentity();
		mPos1 = initialpos;
		setNode(entity);
	}

	virtual ~QuakeBodyMotionState()
	{
	}
	
	// Custom new/delete operators to ensure 16-bit alignment
	void* operator new(size_t i)
	{
		return _mm_malloc(i, 16);
	}

	void operator delete(void* p)
	{
		_mm_free(p);
	}

	void setNode(edict_t *node)
	{
		mVisibleobj = node;
		setWorldTransform(mPos1);
	}

	virtual void getWorldTransform(btTransform &worldTrans) const
	{
		worldTrans = mPos1;
	}

	bool canReset;
	btTransform resetPosition;
	virtual void setResetPosition (const btTransform &worldTrans)
	{
		canReset = true;
		resetPosition = worldTrans;
	}

	virtual void setWorldTransform(const btTransform &worldTrans)
	{
		if (mVisibleobj == null)
			return; // silently return before we set a node

		btQuaternion rot = worldTrans.getRotation();
		btVector3 pos = worldTrans.getOrigin();

		Vec3Copy(mVisibleobj->s.origin, mVisibleobj->s.oldOrigin);

		mVisibleobj->s.origin[0] = pos.x();
		mVisibleobj->s.origin[1] = pos.y();
		mVisibleobj->s.origin[2] = pos.z();

		var orient = ConvertQuat(rot);	
		Vec4Set(mVisibleobj->s.quat, orient.getX(), orient.getY(), orient.getZ(), orient.getW());

		gi.linkentity(mVisibleobj);
		mPos1 = worldTrans;
	}

protected:
	edict_t *mVisibleobj;
	btTransform mPos1;
};


struct physicsEntity
{
	myRigidBody		*body;
	edict_t			*refEntity;

	physicsEntity(myRigidBody* _body)
	{
		body = _body;
		refEntity = G_Spawn();
		refEntity->movetype = MOVETYPE_NONE;
		refEntity->solid = SOLID_NOT;
		refEntity->s.type |= ET_QUATERNION;

		_body->setUserPointer(refEntity);
	}

	physicsEntity(myRigidBody* _body, edict_t *_entity)
	{
		body = _body;
		refEntity = _entity;
		refEntity->movetype = MOVETYPE_NONE;
		refEntity->solid = SOLID_NOT;
		refEntity->s.type |= ET_QUATERNION;

		_body->setUserPointer(refEntity);
	}

	physicsEntity()
	{
	}
};

std::vector<physicsEntity> tempBody;

/*
 *
 * BMODEL LISTS
 * 
 */

struct BModel
{
	int				Index;
	btCompoundShape	*Shape;
	TList<edict_t*>	Users;
	bool			Rigid;
};
	
TList<BModel> bmodels;
BModel *currentModel;
vec3_t modelOffset;

void G_GetBModelVertices (TList<btVector3> vertices)
{
	var tempShape = new btConvexHullShape((const float*)&vertices[0], vertices.Count());

	btScalar scaling(1);

	tempShape->setLocalScaling(btVector3(scaling,scaling,scaling));
	btTransform tf;
	tf.setIdentity();
	currentModel->Shape->addChildShape(btTransform::getIdentity(), tempShape);
}

btCompoundShape *GetBModelShape (int index)
{
	Vec3Clear(modelOffset);

	for (uint32 i = 0; i < bmodels.Count(); ++i)
		if (bmodels[i].Index == index)
			return bmodels[i].Shape;

	BModel model;
	model.Index = index;
	model.Shape = new btCompoundShape();
	model.Shape->setUserPointer(reinterpret_cast<void*>(bmodels.Count()));
	currentModel = &model;
	gi.R_GetBModelVertices(index, G_GetBModelVertices);
	bmodels.Add(model);

	return model.Shape;
}

btCompoundShape *GetBModelShape (vec3_t origin, int index)
{
	Vec3Copy(origin, modelOffset);

	for (uint32 i = 0; i < bmodels.Count(); ++i)
		if (bmodels[i].Index == index)
			return bmodels[i].Shape;

	BModel model;
	model.Index = index;
	model.Shape = new btCompoundShape();
	model.Shape->setUserPointer(reinterpret_cast<void*>(bmodels.Count()));
	currentModel = &model;
	gi.R_GetBModelVertices(index, G_GetBModelVertices);
	bmodels.Add(model);

	return model.Shape;
}

void Phys_SetBModelOnEntity (edict_t *entity, btCompoundShape *shape)
{
	int index = reinterpret_cast<int>(shape->getUserPointer());
	BModel &model = bmodels[index];

	btVector3 localInertia(0,0,0);

	btTransform startTransform;
	startTransform.setIdentity();
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo rbInfo(0,myMotionState,shape,localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	body->setRestitution(1.0f);
	body->setFriction(1.0f);

	if (entity != null)
	{
		body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
		body->setActivationState(DISABLE_DEACTIVATION);
		model.Users.Add(entity);
		entity->physicBody = body;
	}

	physicsWorld->addCollisionObject(body);
}

int IndexFromModelIndex (const String &modelIndex)
{
	return atoi(modelIndex.Substring(1).CString()) - 1;
}

btQuaternion EulerToQuat (vec3_t angles)
{
	btQuaternion q;
	q.setEulerZYX(DEG2RAD(angles[1]), DEG2RAD(angles[0]), DEG2RAD(angles[2]));
	return q;
}

void UpdateBModels ()
{
	for (uint32 i = 0; i < bmodels.Count(); ++i)
	{
		BModel &model = bmodels[i];

		for (uint32 x = 0; x < model.Users.Count(); ++x)
		{
			var user = model.Users[x];
			var body = (btRigidBody*)user->physicBody;

			btTransform trans;
			trans.setIdentity();
			trans.setOrigin(btVector3(user->s.origin[0], user->s.origin[1], user->s.origin[2]));
			trans.setRotation(EulerToQuat(user->s.angles));
			body->getMotionState()->setWorldTransform(trans);
		}
	}
}

const float WORLDSCALE = 1;

btRigidBody *worldBody;

std::vector<std::pair<std::string, btConvexHullShape*>> shapes;
btConvexHullShape *shapePtr;

static void Phys_ReceiveModel (vec3_t *vertice, int *indices, int num_indices)
{
	for (int i = 0; i < num_indices; ++i)
		shapePtr->addPoint(btVector3(vertice[indices[i]][0] * WORLDSCALE, vertice[indices[i]][1] * WORLDSCALE, vertice[indices[i]][2] * WORLDSCALE));
}

btConvexHullShape *GetConvexShape(const char *model)
{
	for (size_t i = 0; i < shapes.size(); ++i)
	{
		if (_stricmp(shapes[i].first.c_str(), model) == 0)
			return shapes[i].second;
	}

	var convexShape = new btConvexHullShape();
	convexShape->setLocalScaling(btVector3(1,1,1));
	shapePtr = convexShape;

	gi.R_GetModelVertices(model, Phys_ReceiveModel); 

	shapes.push_back(std::pair<std::string, btConvexHullShape*>(model, convexShape));
	return convexShape;
}

btSphereShape *sphereShape;

btRigidBody*	localCreateRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape)
{
	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

	btRigidBody* body = new btRigidBody(mass,0,shape,localInertia);	
	body->setWorldTransform(startTransform);
	physicsWorld->addRigidBody(body);

	return body;
}

btRigidBody **playerBodies;

void CG_PhysInit ()
{
	sphereShape = new btSphereShape(2.15f * WORLDSCALE);
	physicsConfig = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	physicsDispatcher = new	btCollisionDispatcher(physicsConfig);

	physicsBroadphase = new btAxisSweep3(btVector3(-4096, -4096, -4096), btVector3(4096, 4096, 4096));

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	physicsSolver = new btSequentialImpulseConstraintSolver;

	physicsWorld = new btDiscreteDynamicsWorld(physicsDispatcher, physicsBroadphase, physicsSolver, physicsConfig);
	physicsWorld->getSolverInfo().m_numIterations = 10;

	physicsWorld->setGravity(btVector3(0, 0, -(170 * WORLDSCALE)));

	gi.SV_SetPhysics(physicsWorld);

	Phys_SetBModelOnEntity(null, GetBModelShape(0));

	{
		playerBodies = new btRigidBody*[game.maxclients];

		for (int i = 0; i < game.maxclients; ++i)
		{
			btTransform startTransform;
			startTransform.setIdentity();
			startTransform.setOrigin(btVector3(9999,9999,9999));

			var charShape = new btCapsuleShapeZ(5, 36);
			btRigidBody *kineBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(0, new btDefaultMotionState(startTransform), charShape));
			kineBody->setCollisionFlags( kineBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
			kineBody->setActivationState(DISABLE_DEACTIVATION);
			physicsWorld->addRigidBody(kineBody);

			playerBodies[i] = kineBody;
		}
	}
}

void UpdateWheels();

btClock realClock;
void CG_PhysStep()
{	
	if (physicsWorld != NULL)
	{
		var x = realClock.getTimeMicroseconds();
		realClock.reset();

		physicsWorld->stepSimulation(x / (1000000.0f / 2), 4, 1.0f / 60.0f);
	}

	for (int i = 0; i < game.maxclients; ++i)
	{
		btTransform trans;
		trans.setIdentity();
		if (!g_edicts[i+1].client->pers.connected || g_edicts[i+1].deadflag)
			trans.setOrigin(btVector3(9999, 9999, 9999));
		else
			trans.setOrigin(btVector3(g_edicts[i+1].s.origin[0], g_edicts[i+1].s.origin[1], g_edicts[i+1].s.origin[2]));

		playerBodies[i]->getMotionState()->setWorldTransform(trans);
	}

	UpdateBModels();

	for (uint32 i = 0; i < tempBody.size(); ++i)
	{
		btTransform fx;
		tempBody[i].body->getMotionState()->getWorldTransform(fx);

		btVector3 vec = fx.getOrigin() / WORLDSCALE;
		var entity = tempBody[i].refEntity;

		if (entity->touch)
		{
			var vel = tempBody[i].body->getLinearVelocity() / (WORLDSCALE * 15);

			vec3_t start, end;

			Vec3Copy(vec, start);
			var endV = vec + vel;
			Vec3Copy(endV, end);

			cmTrace_t trace = gi.trace(start, NULL, NULL, end, entity, CONTENTS_SOLID|CONTENTS_WINDOW|CONTENTS_MONSTER|CONTENTS_DEADMONSTER);

			if (trace.fraction < 1.0)
			{
				if ((trace.ent == Q_World && !tempBody[i].body->getLinearVelocity().isZero()) || (trace.ent != Q_World))
					entity->touch(entity, trace.ent, &trace.plane, trace.surface);
			}
		}

		if (tempBody.size() == 0)
			continue;

		if (entity->physicBody == NULL)
			continue;

		entity->s.type |= ET_QUATERNION;

		var pc = gi.pointcontents(entity->s.origin);

		vec3_t tempOrg;
		Vec3Copy(entity->s.origin, tempOrg);
		tempOrg[2] -= 10;

		if (pc & CONTENTS_MASK_WATER)
		{
			vec3_t temp;
			Vec3Copy(entity->s.origin, temp);

			temp[2] -= 2048;
			cmTrace_t downTrace = gi.trace(entity->s.origin, vec3Origin, vec3Origin, temp, NULL, CONTENTS_SOLID);

			temp[2] += 4096;
			cmTrace_t upTrace = gi.trace(entity->s.origin, vec3Origin, vec3Origin, temp, NULL, CONTENTS_SOLID);

			vec3_t temp2;
			Vec3Copy(temp, temp2);
			temp2[2] -= 8096;
			cmTrace_t downFromUpTrace = gi.trace(temp, vec3Origin, vec3Origin, temp2, NULL, CONTENTS_MASK_WATER);

			float entDistanceToLip = downFromUpTrace.endPos[2] - entity->s.origin[2];

			tempBody[i].body->setGravity(btVector3(0, 0, 60 * (entDistanceToLip / 100)));
			tempBody[i].body->setDamping(tempBody[i].body->getNormalLinearDamping() + 0.35f, tempBody[i].body->getNormalAngularDamping() + 0.35f);
		}
		else if (gi.pointcontents(tempOrg) & CONTENTS_MASK_WATER)
		{
			tempBody[i].body->setGravity(btVector3(0, 0, -(34 * WORLDSCALE)));
			tempBody[i].body->setDamping(tempBody[i].body->getNormalLinearDamping() + 0.35f, tempBody[i].body->getNormalAngularDamping() + 0.35f);
		}
		else
		{
			tempBody[i].body->setDamping(tempBody[i].body->getNormalLinearDamping(), tempBody[i].body->getNormalAngularDamping());
			tempBody[i].body->setGravity(btVector3(0, 0, -(170 * WORLDSCALE)));

		}

		gi.linkentity(entity);
	}
}

void PhysExplosion (vec3_t origin, float dist, float scale, float force)
{
	vec3_t org;
	Vec3Scale (origin, WORLDSCALE, org);

	vec3_t mins, maxs;
	for (int x = 0; x < 3; ++x)
	{
		mins[x] = org[x] - (dist * WORLDSCALE);
		maxs[x] = org[x] + (dist * WORLDSCALE);
	}

	testQuery callback;
	Vec3Copy(org, callback.center);
	physicsWorld->getBroadphase()->aabbTest(btVector3(mins[0], mins[1], mins[2]), btVector3(maxs[0], maxs[1], maxs[2]), callback);

	for (size_t i = 0; i < callback.bodies.size(); ++i)
	{
		std::pair<btRigidBody*, float> pair = callback.bodies[i];

		float maxDist = (dist * WORLDSCALE);

		if (pair.second > (maxDist))
			continue;

		var y = pair.first->getCenterOfMassPosition() - btVector3(callback.center[0], callback.center[1], callback.center[2]);

		y.normalize();
		y *= ((maxDist - pair.second) / maxDist) * (force);

		var len = y.length();
		if (len < 0 || len > abs(force))
			continue;

		pair.first->activate(true);
		pair.first->applyCentralImpulse(y);
	}

	Vec3Clear(callback.center);
}

/*
=================
fire_grenade
=================
*/
static void PhysGrenade_Explode (edict_t *ent)
{
	int			mod;

	if (ent->owner->client)
		PlayerNoise(ent->owner, ent->s.origin, PNOISE_IMPACT);

	//FIXME: if we are onground then raise our Z just a bit since we are a point?
	if (ent->enemy)
	{
		float	points;
		vec3_t	v;
		vec3_t	dir;

		Vec3Add (ent->enemy->mins, ent->enemy->maxs, v);
		Vec3MA (ent->enemy->s.origin, 0.5, v, v);
		Vec3Subtract (ent->s.origin, v, v);
		points = ent->dmg - 0.5 * Vec3Length (v);
		Vec3Subtract (ent->enemy->s.origin, ent->s.origin, dir);
		if (ent->spawnflags & 1)
			mod = MOD_HANDGRENADE;
		else
			mod = MOD_GRENADE;
		T_Damage (ent->enemy, ent, ent->owner, dir, ent->s.origin, vec3Origin, (int)points, (int)points, DAMAGE_RADIUS, mod);
	}

	if (ent->spawnflags & 2)
		mod = MOD_HELD_GRENADE;
	else if (ent->spawnflags & 1)
		mod = MOD_HG_SPLASH;
	else
		mod = MOD_G_SPLASH;
	T_RadiusDamage(ent, ent->owner, ent->dmg, ent->enemy, ent->dmg_radius, mod);

	Vec3MA (ent->s.origin, -0.02, ent->velocity, ent->s.origin);
	ConvertToEntityEvent(ent, EV_EXPLOSION, entityParms_t().PushByte((ent->waterlevel) ? 1 : 0));

	PhysExplosion(ent->s.origin, 290.0f, 0.85f, 65.0f);
}

void RemovePhysBody (edict_t *ent)
{
	size_t i;
	for (i = 0; i < tempBody.size(); ++i)
	{
		if (tempBody[i].refEntity == ent)
			break;
	}

	tempBody.erase(tempBody.begin()+i);
	physicsWorld->removeRigidBody((btRigidBody*)ent->physicBody);
	delete ((btRigidBody*)ent->physicBody)->getMotionState();
	ent->physicBody = NULL;
}

static void PhysGrenade_Touch (edict_t *ent, edict_t *other, plane_t *plane, cmBspSurface_t *surf)
{
	if (other == ent->owner)
		return;

	if (surf && (surf->flags & SURF_TEXINFO_SKY))
	{
		G_FreeEdict (ent);
		return;
	}

	if (!other->takedamage)
	{
		if (ent->spawnflags & 1)
		{
			if (random() > 0.5)
				gi.sound (ent, CHAN_VOICE, gi.soundindex ("weapons/hgrenb1a.wav"), 1, ATTN_NORM, 0);
			else
				gi.sound (ent, CHAN_VOICE, gi.soundindex ("weapons/hgrenb2a.wav"), 1, ATTN_NORM, 0);
		}
		else
		{
			gi.sound (ent, CHAN_VOICE, gi.soundindex ("weapons/grenlb1b.wav"), 1, ATTN_NORM, 0);
		}
		return;
	}

	ent->enemy = other;
	PhysGrenade_Explode (ent);
}

physicsEntity AllocPhysEntity (vec3_t origin, vec3_t angles, vec3_t velocity, btCollisionShape *shape, float mass)
{
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(origin[0] * WORLDSCALE, origin[1] * WORLDSCALE, origin[2] * WORLDSCALE));

	shape->calculateLocalInertia(mass,btVector3());

	var q = btQuaternion::getIdentity();
	q.setEulerZYX(DEG2RAD(angles[1]), DEG2RAD(angles[0]), DEG2RAD(angles[2]));

	startTransform.setRotation(q);

	btVector3 localInertia(0,0,0);
	if (mass != 0.0f)
		shape->calculateLocalInertia(mass,localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	QuakeBodyMotionState* myMotionState = new QuakeBodyMotionState(startTransform, null);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
	myRigidBody *body = new myRigidBody(rbInfo);	

	physicsWorld->addRigidBody(body);

	physicsEntity entity(body);
	myMotionState->setNode(entity.refEntity);
	tempBody.push_back(entity);

	if (velocity[0] != 0 || velocity[1] != 0 || velocity[2] != 0)
		body->applyImpulse(btVector3((velocity[0] / 5) * WORLDSCALE, (velocity[1] / 5) * WORLDSCALE, (velocity[2] / 5) * WORLDSCALE), btVector3(0, 0, 0));

	entity.refEntity->physicBody = entity.body;
	Vec3Copy(origin, entity.refEntity->s.origin);
	Vec3Copy(origin, entity.refEntity->s.oldOrigin);

	return entity;
}

physicsEntity AllocPhysEntity (vec3_t origin, vec3_t angles, vec3_t velocity, const char *model, float mass, bool index = true)
{
	var alloc = AllocPhysEntity (origin, angles, velocity, GetConvexShape(model), mass);
	if (index)
		alloc.refEntity->s.modelIndex = gi.modelindex((char*)model);
	return alloc;
}

physicsEntity AllocPhysEntityFromEntity (edict_t *ent, btCollisionShape *shape, float mass, bool awake)
{
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(ent->s.origin[0] * WORLDSCALE, ent->s.origin[1] * WORLDSCALE, ent->s.origin[2] * WORLDSCALE));

	shape->calculateLocalInertia(mass,btVector3());

	var q = btQuaternion::getIdentity();
	q.setEulerZYX(DEG2RAD(ent->s.angles[1]), DEG2RAD(ent->s.angles[0]), DEG2RAD(ent->s.angles[2]));

	startTransform.setRotation(q);

	btVector3 localInertia(0,0,0);
	if (mass != 0.0f)
		shape->calculateLocalInertia(mass,localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	QuakeBodyMotionState* myMotionState = new QuakeBodyMotionState(startTransform, ent);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
	myRigidBody *body = new myRigidBody(rbInfo);

	body->setActivationState((!awake) ? ISLAND_SLEEPING : ACTIVE_TAG);

	physicsWorld->addRigidBody(body);

	physicsEntity entity(body, ent);
	tempBody.push_back(entity);

	if (ent->velocity[0] != 0 || ent->velocity[1] != 0 || ent->velocity[2] != 0)
		body->applyImpulse(btVector3((ent->velocity[0] / 5) * WORLDSCALE, (ent->velocity[1] / 5) * WORLDSCALE, (ent->velocity[2] / 5) * WORLDSCALE), btVector3(0, 0, 0));

	entity.refEntity->physicBody = entity.body;

	myMotionState->setResetPosition(body->getWorldTransform());

	return entity;
}

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

const int CONSTRAINT_DEBUG_SIZE = 3;
const float RagdollScale = 39;

edict_t *ThrowPhysicsGibInt (edict_t *self, char *gibname, int damage, int type);
class RagDoll
{
public:
	enum
	{
		JOINT_PELVIS_SPINE = 0,
		JOINT_SPINE_HEAD,

		JOINT_LEFT_HIP,
		JOINT_LEFT_KNEE,

		JOINT_RIGHT_HIP,
		JOINT_RIGHT_KNEE,

		JOINT_LEFT_SHOULDER,
		JOINT_LEFT_ELBOW,

		JOINT_RIGHT_SHOULDER,
		JOINT_RIGHT_ELBOW,

		JOINT_COUNT
	};

	btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shapes[RAG_COUNT];
	myRigidBody* m_bodies[RAG_COUNT];
	btTypedConstraint* m_joints[JOINT_COUNT];

	myRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, int part)
	{
		mass /= 3;
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			m_shapes[part]->calculateLocalInertia(mass,localInertia);

		btMotionState *myMotionState;
		myMotionState = new QuakeBodyMotionState(startTransform, null);

		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,m_shapes[part],localInertia);
		myRigidBody* body = new myRigidBody(rbInfo);

		physicsEntity entity(body);
		entity.refEntity->s.type |= ET_RAGDOLL;
		entity.refEntity->physicBody = body;
		entity.refEntity->s.modelIndex = playerNum;
		entity.refEntity->s.skinNum = part;
		((QuakeBodyMotionState*)myMotionState)->setNode(entity.refEntity);
		tempBody.push_back(entity);

		m_ownerWorld->addRigidBody(body);

		return body;
	}

	edict_t *chestBody;
	int playerNum;

	RagDoll (int playerNumber, btDynamicsWorld* ownerWorld, const btVector3& positionOffset, vec3_t angles, vec3_t velocity, float scale_ragdoll)
		: m_ownerWorld (ownerWorld)
	{
		playerNum = playerNumber;
		// Setup the geometry
		m_shapes[RAG_PELVIS] = new btCapsuleShape(scale_ragdoll*btScalar(0.15), scale_ragdoll*btScalar(0.10));
		m_shapes[RAG_SPINE] = new btCapsuleShape(scale_ragdoll*btScalar(0.15), scale_ragdoll*btScalar(0.28));
		m_shapes[RAG_HEAD] = new btCapsuleShape(scale_ragdoll*btScalar(0.10), scale_ragdoll*btScalar(0.05));
		m_shapes[RAG_LUPPERLEG] = new btCapsuleShape(scale_ragdoll*btScalar(0.07), scale_ragdoll*btScalar(0.34));
		m_shapes[RAG_LLOWERLEG] = new btCapsuleShape(scale_ragdoll*btScalar(0.05), scale_ragdoll*btScalar(0.36));
		m_shapes[RAG_RUPPERLEG] = new btCapsuleShape(scale_ragdoll*btScalar(0.07), scale_ragdoll*btScalar(0.34));
		m_shapes[RAG_RLOWERLEG] = new btCapsuleShape(scale_ragdoll*btScalar(0.05), scale_ragdoll*btScalar(0.36));
		m_shapes[RAG_LUPPERARM] = new btCapsuleShape(scale_ragdoll*btScalar(0.05), scale_ragdoll*btScalar(0.10));
		m_shapes[RAG_LLOWERARM] = new btCapsuleShape(scale_ragdoll*btScalar(0.04), scale_ragdoll*btScalar(0.25));
		m_shapes[RAG_RUPPERARM] = new btCapsuleShape(scale_ragdoll*btScalar(0.05), scale_ragdoll*btScalar(0.10));
		m_shapes[RAG_RLOWERARM] = new btCapsuleShape(scale_ragdoll*btScalar(0.04), scale_ragdoll*btScalar(0.25));

		// Setup all the rigid bodies
		btTransform offset; offset.setIdentity();
		offset.setOrigin(positionOffset);
		offset.setRotation(EulerToQuat(angles));

		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(scale_ragdoll*btScalar(0.), scale_ragdoll*btScalar(1.), scale_ragdoll*btScalar(0.)));
		m_bodies[RAG_PELVIS] = localCreateRigidBody(btScalar(1.), offset*transform, RAG_PELVIS);

		transform.setIdentity();
		transform.setOrigin(btVector3(scale_ragdoll*btScalar(0.), scale_ragdoll*btScalar(1.2), scale_ragdoll*btScalar(0.)));
		m_bodies[RAG_SPINE] = localCreateRigidBody(btScalar(1.), offset*transform, RAG_SPINE);

		chestBody = (edict_t*)m_bodies[RAG_SPINE]->getUserPointer();
		chestBody->team = reinterpret_cast<char*>(this);

		transform.setIdentity();
		transform.setOrigin(btVector3(scale_ragdoll*btScalar(0.), scale_ragdoll*btScalar(1.6), scale_ragdoll*btScalar(0.)));
		m_bodies[RAG_HEAD] = localCreateRigidBody(btScalar(1.), offset*transform, RAG_HEAD);

		transform.setIdentity();
		transform.setOrigin(btVector3(scale_ragdoll*btScalar(-0.18), scale_ragdoll*btScalar(0.65), scale_ragdoll*btScalar(0.)));
		m_bodies[RAG_LUPPERLEG] = localCreateRigidBody(btScalar(1.), offset*transform, RAG_LUPPERLEG);

		transform.setIdentity();
		transform.setOrigin(btVector3(scale_ragdoll*btScalar(-0.18), scale_ragdoll*btScalar(0.2), scale_ragdoll*btScalar(0.)));
		m_bodies[RAG_LLOWERLEG] = localCreateRigidBody(btScalar(1.), offset*transform, RAG_LLOWERLEG);

		transform.setIdentity();
		transform.setOrigin(btVector3(scale_ragdoll*btScalar(0.18),scale_ragdoll* btScalar(0.65), scale_ragdoll*btScalar(0.)));
		m_bodies[RAG_RUPPERLEG] = localCreateRigidBody(btScalar(1.), offset*transform, RAG_RUPPERLEG);

		transform.setIdentity();
		transform.setOrigin(btVector3(scale_ragdoll*btScalar(0.18), scale_ragdoll*btScalar(0.2), scale_ragdoll*btScalar(0.)));
		m_bodies[RAG_RLOWERLEG] = localCreateRigidBody(btScalar(1.), offset*transform, RAG_RLOWERLEG);

		transform.setIdentity();
		transform.setOrigin(btVector3(scale_ragdoll*btScalar(-0.35), scale_ragdoll*btScalar(1.45), scale_ragdoll*btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,M_PI_2);
		m_bodies[RAG_LUPPERARM] = localCreateRigidBody(btScalar(1.), offset*transform, RAG_LUPPERARM);

		transform.setIdentity();
		transform.setOrigin(btVector3(scale_ragdoll*btScalar(-0.7), scale_ragdoll*btScalar(1.45), scale_ragdoll*btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,M_PI_2);
		m_bodies[RAG_LLOWERARM] = localCreateRigidBody(btScalar(1.), offset*transform, RAG_LLOWERARM);

		transform.setIdentity();
		transform.setOrigin(btVector3(scale_ragdoll*btScalar(0.35), scale_ragdoll*btScalar(1.45), scale_ragdoll*btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,-M_PI_2);
		m_bodies[RAG_RUPPERARM] = localCreateRigidBody(btScalar(1.), offset*transform, RAG_RUPPERARM);

		transform.setIdentity();
		transform.setOrigin(btVector3(scale_ragdoll*btScalar(0.7), scale_ragdoll*btScalar(1.45), scale_ragdoll*btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,-M_PI_2);
		m_bodies[RAG_RLOWERARM] = localCreateRigidBody(btScalar(1.), offset*transform, RAG_RLOWERARM);

		// Setup some damping on the m_bodies
		for (int i = 0; i < RAG_COUNT; ++i)
		{
			m_bodies[i]->setNormalDamping(0.05, 0.45);
			m_bodies[i]->setDeactivationTime(0.8);
			m_bodies[i]->setSleepingThresholds(1.6, 2.5);
		
			m_bodies[i]->setLinearVelocity(btVector3(velocity[0], velocity[1], velocity[2]));
		}
		
		// Now setup the constraints
		btHingeConstraint* hingeC;
		btConeTwistConstraint* coneC;

		btTransform localA, localB;

	{
		localA.setIdentity(); localB.setIdentity();

		localA.getBasis().setEulerZYX(0,SIMD_HALF_PI,0);
		localA.setOrigin(btVector3(btScalar(0.), btScalar(0.15*scale_ragdoll), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,SIMD_HALF_PI,0);
		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.15*scale_ragdoll), btScalar(0.)));
		var joint6DOF =  new btGeneric6DofConstraint (*m_bodies[RAG_PELVIS], *m_bodies[RAG_SPINE], localA, localB,true);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_PI*0.2,-SIMD_EPSILON,-SIMD_PI*0.3));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.2,SIMD_EPSILON,SIMD_PI*0.6));
#endif
		m_joints[JOINT_PELVIS_SPINE] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_PELVIS_SPINE], true);
	}


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI_2); localA.setOrigin(scale_ragdoll*btVector3(btScalar(0.), btScalar(0.05), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(scale_ragdoll*btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[RAG_SPINE], *m_bodies[RAG_HEAD], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, M_PI_2);
		m_joints[JOINT_SPINE_HEAD] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_SPINE_HEAD], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,-M_PI_4*5); localA.setOrigin(scale_ragdoll*btVector3(btScalar(-0.09), btScalar(-0.10), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,-M_PI_4*5); localB.setOrigin(scale_ragdoll*btVector3(btScalar(0.), btScalar(0.125), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[RAG_PELVIS], *m_bodies[RAG_LUPPERLEG], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, 0);
		m_joints[JOINT_LEFT_HIP] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_HIP], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(scale_ragdoll*btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(scale_ragdoll*btVector3(btScalar(0.), btScalar(0.100), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[RAG_LUPPERLEG], *m_bodies[RAG_LLOWERLEG], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_LEFT_KNEE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_KNEE], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI_4); localA.setOrigin(scale_ragdoll*btVector3(btScalar(0.09), btScalar(-0.10), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_4); localB.setOrigin(scale_ragdoll*btVector3(btScalar(0.), btScalar(0.125), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[RAG_PELVIS], *m_bodies[RAG_RUPPERLEG], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, 0);
		m_joints[JOINT_RIGHT_HIP] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_HIP], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(scale_ragdoll*btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(scale_ragdoll*btVector3(btScalar(0.), btScalar(0.100), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[RAG_RUPPERLEG], *m_bodies[RAG_RLOWERLEG], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_RIGHT_KNEE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_KNEE], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI); localA.setOrigin(scale_ragdoll*btVector3(btScalar(-0.2), btScalar(0.02), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(scale_ragdoll*btVector3(btScalar(0.), btScalar(-0.08), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[RAG_SPINE], *m_bodies[RAG_LUPPERARM], localA, localB);
		coneC->setLimit(M_PI_2, M_PI_2, 0);
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_joints[JOINT_LEFT_SHOULDER] = coneC;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_SHOULDER], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(scale_ragdoll*btVector3(btScalar(0.), btScalar(0.11), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(scale_ragdoll*btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[RAG_LUPPERARM], *m_bodies[RAG_LLOWERARM], localA, localB);
		//		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_LEFT_ELBOW] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_ELBOW], true);



		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,0); localA.setOrigin(scale_ragdoll*btVector3(btScalar(0.2), btScalar(0.02), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(scale_ragdoll*btVector3(btScalar(0.), btScalar(-0.08), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[RAG_SPINE], *m_bodies[RAG_RUPPERARM], localA, localB);
		coneC->setLimit(M_PI_2, M_PI_2, 0);
		m_joints[JOINT_RIGHT_SHOULDER] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_SHOULDER], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(scale_ragdoll*btVector3(btScalar(0.), btScalar(0.11), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(scale_ragdoll*btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[RAG_RUPPERARM], *m_bodies[RAG_RLOWERARM], localA, localB);
		//		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_RIGHT_ELBOW] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_ELBOW], true);

		for (int i = 0; i < RAG_COUNT; ++i)
			m_bodies[i]->clearForces();
	}

	void DestroyBodyPart (myRigidBody *body)
	{
		var gib = ThrowPhysicsGibInt((edict_t*)body->getUserPointer(), "models/objects/gibs/sm_meat/tris.md2", 0, GIB_ORGANIC);
		((myRigidBody*)gib->physicBody)->setLinearVelocity(body->getLinearVelocity());
		((myRigidBody*)gib->physicBody)->setAngularVelocity(body->getAngularVelocity());

		for (int i = 0; i < JOINT_COUNT; ++i)
		{
			if (m_joints[i] == null)
				continue;

			if (&m_joints[i]->getRigidBodyA() == body ||
				&m_joints[i]->getRigidBodyB() == body)
			{
				m_joints[i]->getRigidBodyA().activate();
				m_joints[i]->getRigidBodyB().activate();
				m_ownerWorld->removeConstraint(m_joints[i]);
				delete m_joints[i];
				m_joints[i] = null;
			}
		}

		for (int i = 0; i < RAG_COUNT; ++i)
		{
			if (m_bodies[i] == body)
			{
				G_FreeEdict((edict_t*)m_bodies[i]->getUserPointer());
				delete m_bodies[i];
				m_bodies[i] = null;
				break;
			}
		}
	}

	int PieceIndex (myRigidBody *body)
	{
		for (int i = 0; i < RAG_COUNT; ++i)
		{
			if (m_bodies[i] == body)
				return i;
		}

		return -1;
	}

	virtual	~RagDoll ()
	{
		int i;

		// Remove all constraints
		for ( i = 0; i < JOINT_COUNT; ++i)
		{
			if (!m_joints[i])
				continue;

			m_ownerWorld->removeConstraint(m_joints[i]);
			delete m_joints[i]; m_joints[i] = 0;
		}

		// Remove all bodies and shapes
		for ( i = 0; i < RAG_COUNT; ++i)
		{
			delete m_shapes[i]; m_shapes[i] = 0;

			if (!m_bodies[i])
				continue;

			var edict = (edict_t*)m_bodies[i]->getUserPointer();
			G_FreeEdict(edict);
			
			delete m_bodies[i]; m_bodies[i] = 0;
		}
	}
};

#include <queue>

std::queue<RagDoll*> ragdolls;

void FreeRagdoll (RagDoll *raggy)
{
	delete raggy;
}

edict_t *ClosestRagdollPiece (RagDoll *doll, vec3_t pos)
{
	float bestDist = 999999999999;
	edict_t *bestPiece = null;

	for (int i = 0; i < RAG_COUNT; ++i)
	{
		var piece = (edict_t*)doll->m_bodies[i]->getUserPointer();
		var dist = Vec3DistSquared(piece->s.origin, pos);

		var ent = CreateEntityEvent(EV_DEBUGTRAIL, entityParms_t(), piece->s.origin, vec3Origin, true);
		ent->s.color = colorb(255, 255, 255, 255);
		Vec3Copy(pos, ent->s.oldOrigin);

	
		if (dist < bestDist)
		{
			bestPiece = piece;
			bestDist = dist;
		}
	}

	gi.dprintf ("%i\n", doll->PieceIndex((myRigidBody*)bestPiece->physicBody));
	return bestPiece;
}

void UpdateChaseCar(edict_t *ent)
{
	ent->client->ps.pMove.pmType = PMT_FREEZE;

	var org = ent->client->chaseEntity->s.origin;
	Vec3Copy(org, ent->s.origin);

	vec3_t viewAngles = {0, 0, 0};
	var quat = ent->client->chaseEntity->s.quat;
	var cq = ConvertQuat(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
	mat3x3_t mat;
	Quat_Matrix3(cq, mat);
	vec3_t euler;
	Matrix3_Angles(mat, euler);
	euler[0] -= 90;
	euler[1] += 90;
	Vec3Copy (euler, viewAngles);

	for (int i = 0 ; i < 3; i++)
		ent->client->ps.pMove.deltaAngles[i] = ANGLE2SHORT(viewAngles[i] - ent->client->resp.cmd_angles[i]);

	Vec3Copy(viewAngles, ent->client->ps.viewAngles);
	Vec3Copy(viewAngles, ent->client->v_angle);

	ent->viewheight = 0;
	ent->client->ps.pMove.pmFlags |= PMF_NO_PREDICTION;
	gi.linkentity(ent);
}

RagDoll *latestRagdoll;
void SetAsRagdoll (edict_t *player)
{
	vec3_t angles;
	Vec3Copy(player->s.angles, angles);
	angles[2] = -angles[0] + 90;
	angles[1] -= 90;
	angles[0] = 0;

	vec3_t origin;
	Vec3Set (origin, player->s.origin[0], player->s.origin[1], player->s.origin[2] - 28);

	float sin = sinf(DEG2RAD(player->s.angles[0]));
	vec3_t fwd, rgt, up;
	Angles_Vectors(player->s.angles, fwd, rgt, up);
	Vec3MA(origin, -sin * 35, fwd, origin);
	
	var raggy = new RagDoll(player->s.number, physicsWorld, btVector3(origin[0], origin[1], origin[2]), angles, player->velocity, 45);
	latestRagdoll = raggy;

	if (player->client)
		player->client->chaseEntity = (edict_t*)raggy->m_bodies[RAG_HEAD]->getUserPointer();

	ragdolls.push(raggy);
	if (ragdolls.size() > 4)
	{
		FreeRagdoll(ragdolls.front());
		ragdolls.pop();
	}

	for (int i = 0; i < RAG_COUNT; ++i)
		((edict_t*)raggy->m_bodies[i]->getUserPointer())->enemy = (edict_t*)raggy;

	//raggy->DestroyBodyPart(raggy->m_bodies[RAG_SPINE]);
}

void KillRag (edict_t *self, edict_t *inflictor, edict_t *attacker, int damage, vec3_t point)
{
	SetAsRagdoll(self);

	if (damage > 200)
		damage = 200;

	//ClosestRagdollPiece(latestRagdoll, point);
	for (int i = 0; i < RAG_COUNT; ++i)
	{
		((edict_t*)latestRagdoll->m_bodies[i]->getUserPointer())
			->s.modelIndex = self->enemy->s.number;

		var body = latestRagdoll->m_bodies[i];
		var y = body->getCenterOfMassPosition() - btVector3(point[0], point[1], point[2]);

		y.normalize();
		y *= ((150 - Vec3Dist(point, body->getCenterOfMassPosition())) / 150) * (damage / 2);

		var len = y.length();
		if (len < 0 || len > abs(damage))
			continue;

		body->activate(true);
		body->applyCentralImpulse(y);
	}
}

void CG_PhysAdd (edict_t *player)
{
	edict_t *newEntity = G_Spawn();
	newEntity->s.modelIndex = gi.modelindex("players/male/tris.md2");
	newEntity->solid = SOLID_BBOX;
	Vec3Copy (player->s.origin, newEntity->s.origin);
	Vec3Copy (player->s.angles, newEntity->s.angles);
	newEntity->takedamage = DAMAGE_YES;
	Vec3Set (newEntity->mins, -16, -16, -24);
	Vec3Set (newEntity->maxs,  16,  16,  32);
	newEntity->health = 1;
	newEntity->dmg = player->s.number;
	newEntity->die = KillRag;
	newEntity->enemy = player;

	gi.linkentity(newEntity);
}

void fire_physgrenade (edict_t *self, vec3_t start, vec3_t aimdir, int damage, int speed, float timer, float damage_radius)
{
	vec3_t org;
	vec3_t vel;
	vec3_t	dir;
	vec3_t	forward, right, up;

	VecToAngles (aimdir, dir);
	Angles_Vectors (dir, forward, right, up);

	Vec3Copy (start, org);
	Vec3Scale (aimdir, speed * 1.45f, vel);
	Vec3MA (vel, 170 + crandom() * 10.0, up, vel);
	Vec3MA (vel, crandom() * 10.0, right, vel);

	var body = AllocPhysEntity(org, dir, vel, "models/physics/grenade/tris.md2", 0.65f);
	body.body->setFriction(0.25f);
	body.body->setRestitution(0.45f);

	Vec3Copy(start, body.refEntity->s.origin);
	Vec3Copy(start, body.refEntity->s.oldOrigin);
	body.refEntity->s.effects |= EF_GRENADE;
	Vec3Clear (body.refEntity->mins);
	Vec3Clear (body.refEntity->maxs);
	body.refEntity->owner = self;
	body.refEntity->touch = PhysGrenade_Touch;
	body.refEntity->nextthink = level.time + timer;
	body.refEntity->think = PhysGrenade_Explode;
	body.refEntity->dmg = damage;
	body.refEntity->dmg_radius = damage_radius;
	body.refEntity->classname = "grenade";

	gi.linkentity (body.refEntity);
}

void fire_physgrenade2 (edict_t *self, vec3_t start, vec3_t aimdir, int damage, int speed, float timer, float damage_radius, bool held)
{
	vec3_t org;
	vec3_t vel;
	vec3_t	dir;
	vec3_t	forward, right, up;

	VecToAngles (aimdir, dir);
	Angles_Vectors (dir, forward, right, up);

	Vec3Copy (start, org);
	Vec3Scale (aimdir, speed, vel);
	Vec3MA (vel, 160 + crandom() *10.0, up, vel);
	Vec3MA (vel, crandom() * 10.0, right, vel);

	var body = AllocPhysEntity(org, dir, vel, sphereShape, 0.35f);
	body.body->setFriction(0.65f);
	body.body->setDamping(0, 0.75f);
	body.body->setNormalDamping(0, 0.75f);
	body.body->setRestitution(0.35f);
	body.refEntity->s.effects |= EF_GRENADE;
	Vec3Clear (body.refEntity);
	Vec3Clear (body.refEntity);
	body.refEntity->s.modelIndex = gi.modelindex("models/objects/grenade2/tris.md2");
	body.refEntity->owner = self;
	body.refEntity->touch = PhysGrenade_Touch;
	body.refEntity->nextthink = level.time + timer;
	body.refEntity->think = PhysGrenade_Explode;
	body.refEntity->dmg = damage;
	body.refEntity->dmg_radius = damage_radius;
	body.refEntity->classname = "hgrenade";
	if (held)
		body.refEntity->spawnflags = 3;
	else
		body.refEntity->spawnflags = 1;
	body.refEntity->s.sound = gi.soundindex("weapons/hgrenc1b.wav");

	if (timer <= 0.0)
		PhysGrenade_Explode (body.refEntity);
	else
	{
		gi.sound (self, CHAN_WEAPON, gi.soundindex ("weapons/hgrent1a.wav"), 1, ATTN_NORM, 0);
		gi.linkentity (body.refEntity);
	}
}

void gib_physics_die (edict_t *self, edict_t *inflictor, edict_t *attacker, int damage, vec3_t point)
{
	G_FreeEdict (self);
}

void VelocityForDamage (int damage, vec3_t v);
void ClipGibVelocity (vec3_t v);

edict_t *ThrowPhysicsGibInt (edict_t *self, char *gibname, int damage, int type)
{
	vec3_t	vd;
	vec3_t	origin;
	vec3_t	size;
	vec3_t vel;

	Vec3Scale (self->size, 0.5, size);
	Vec3Add (self->absMin, size, origin);
	origin[0] = origin[0] + crandom() * size[0];
	origin[1] = origin[1] + crandom() * size[1];
	origin[2] = origin[2] + crandom() * size[2];

	VelocityForDamage (damage, vd);
	Vec3MA (self->velocity, (type == GIB_ORGANIC) ? 0.5f : 1.0f, vd, vel);
	ClipGibVelocity (vel);

	var body = AllocPhysEntity(origin, vec3Origin, vel, gibname, 0.45f);
	body.body->setAngularVelocity(btVector3(crandom()*5, crandom()*5, crandom()*5));
	body.body->setFriction(0.65f);

	body.refEntity->solid = SOLID_NOT;
	body.refEntity->s.effects |= EF_GIB;
	body.refEntity->flags |= FL_NO_KNOCKBACK;
	body.refEntity->takedamage = DAMAGE_YES;
	body.refEntity->die = gib_physics_die;

	if (type == GIB_ORGANIC)
		body.body->setRestitution(0.15f);
	else
		body.body->setRestitution(0.05f);

	body.refEntity->think = G_FreeEdict;
	body.refEntity->nextthink = level.time + 10 + random()*10;

	gi.linkentity (body.refEntity);

	return body.refEntity;
}


void ThrowPhysicsGib (edict_t *self, char *gibname, int damage, int type)
{
	ThrowPhysicsGibInt(self, gibname, damage, type);
}

void Touch_Item (edict_t *ent, edict_t *other, plane_t *plane, cmBspSurface_t *surf);

static void drop_temp_touch (edict_t *ent, edict_t *other, plane_t *plane, cmBspSurface_t *surf)
{
	if (other == ent->owner)
		return;

	Touch_Item (ent, other, plane, surf);
}

static void drop_make_touchable (edict_t *ent)
{
	ent->touch = Touch_Item;
	if (deathmatch->floatVal)
	{
		ent->nextthink = level.time + 29;
		ent->think = G_FreeEdict;
	}
}

edict_t *Drop_Physics_Item (edict_t *ent, gitem_t *item)
{
	vec3_t	forward, right;
	vec3_t	offset;

	vec3_t origin, vel;

	vec3_t mins, maxs;
	Vec3Set (mins, -15, -15, -15);
	Vec3Set (maxs, 15, 15, 15);

	if (ent->client)
	{
		cmTrace_t	trace;

		Angles_Vectors (ent->client->v_angle, forward, right, NULL);
		Vec3Set (offset, 58, 0, -16);
		G_ProjectSource (ent->s.origin, offset, forward, right, origin);
		trace = gi.trace (ent->s.origin, mins, maxs,
			origin, ent, CONTENTS_SOLID);
		Vec3Copy (trace.endPos, origin);

		Vec3Scale (forward, 25, vel);
		vel[2] = 50;

		Vec3MA(vel, 0.5f, ent->velocity, vel);
	}
	else
	{
		Angles_Vectors (ent->s.angles, forward, right, NULL);

		Vec3Scale (forward, 25, vel);
		vel[2] = 100;

		Vec3Copy (ent->s.origin, origin);
	}

	var body = AllocPhysEntity(origin, ent->s.angles, vel, item->world_model, 0.25f, false);
	body.body->setAngularVelocity(btVector3(crandom()*2, crandom()*2, crandom()*2));
	body.body->setFriction(0.65f);
	body.body->setRestitution(0.05f);

	body.refEntity->classname = item->classname;
	body.refEntity->item = item;
	body.refEntity->spawnflags = DROPPED_ITEM;
	Vec3Set (body.refEntity->mins, -15, -15, -15);
	Vec3Set (body.refEntity->maxs, 15, 15, 15);
	body.refEntity->solid = SOLID_TRIGGER;
	body.refEntity->touch = drop_temp_touch;
	body.refEntity->owner = ent;
	body.refEntity->s.modelIndex = ITEM_INDEX(item);
	body.refEntity->s.type |= ET_ITEM;

	body.refEntity->think = drop_make_touchable;
	body.refEntity->nextthink = level.time + 1;

	gi.linkentity (body.refEntity);

	return body.refEntity;
}

void PhysRayCast (vec3_t start, vec3_t end, float kick)
{
	var st = btVector3(start[0], start[1], start[2]), en = btVector3(end[0], end[1], end[2]);
	btCollisionWorld::ClosestRayResultCallback callback(st, en);
	physicsWorld->rayTest(st, en, callback);

	if (callback.m_collisionObject && !callback.m_collisionObject->isStaticOrKinematicObject())
	{
		var y = ((btRigidBody*)callback.m_collisionObject)->getCenterOfMassPosition() - callback.m_hitPointWorld;

		y.normalize();
		y *= kick;

		callback.m_collisionObject->activate(true);
		((btRigidBody*)callback.m_collisionObject)->applyCentralImpulse(y);
	}
}

void PhysRayCastFull (vec3_t start, vec3_t end, float kick)
{
	var st = btVector3(start[0], start[1], start[2]), en = btVector3(end[0], end[1], end[2]);
	btCollisionWorld::AllHitsRayResultCallback callback(st, en);
	physicsWorld->rayTest(st, en, callback);

	for (int i = 0; i < callback.m_collisionObjects.size(); ++i)
	{
		const btCollisionObject *obj = callback.m_collisionObjects[i];

		if (!obj->isStaticOrKinematicObject())
		{
			var y = ((btRigidBody*)obj)->getCenterOfMassPosition() - callback.m_hitPointWorld[i];

			y.normalize();
			y *= kick;

			obj->activate(true);

			var rigd = ((myRigidBody*)obj);
			var entity = static_cast<edict_t*>(rigd->getUserPointer());

			if (static_cast<edict_t*>(rigd->getUserPointer())->s.type & ET_RAGDOLL)
			{
				RagDoll *rag = (RagDoll*)entity->enemy;
				rag->DestroyBodyPart(rigd);
			}
			else
				rigd->applyCentralImpulse(y);
		}
	}
}

void Phys_Wake ()
{
	var &list = physicsWorld->getCollisionObjectArray();

	for (int i = 0; i < list.size(); ++i)
	{
		if (!list[i]->isStaticOrKinematicObject())
			list[i]->activate(true);
	}
}

void SP_func_physics (edict_t *ent)
{
	/*ent->solid = SOLID_BSP;
	gi.setmodel (ent, ent->model);
	gi.linkentity(ent);

	AllocPhysEntityFromEntity(ent, GetBModelShape(ent->s.origin, ent->s.modelIndex-1), 5);
	ent->solid = SOLID_NOT;*/
	G_FreeEdict(ent);
}

void SP_misc_physics (edict_t *ent)
{
	gi.linkentity(ent);
	ent->s.modelIndex = (int)(ent->speed + 1);

	if (!ent->accel)
		ent->accel = 1;
	else if ((int)ent->accel == -1)
		ent->accel = 0;

	AllocPhysEntityFromEntity(ent, GetBModelShape(ent->speed), ent->accel, (ent->decel == 1));
}

void Phys_Reset()
{
	for (uint32 i = 0; i < tempBody.size(); ++i)
	{
		var body = tempBody[i].body;

		QuakeBodyMotionState *state = (QuakeBodyMotionState*)body->getMotionState();

		if (state->canReset)
		{
			body->setLinearVelocity(btVector3(0,0,0));
			body->setAngularVelocity(btVector3(0,0,0));
			body->setWorldTransform(state->resetPosition);
		}
	}
}
