/*
Copyright (C) 1997-2001 Id Software, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or v

See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
*/

// MathLib.h

// ===========================================================================

extern vec2_t	vec2Origin;
extern vec3_t	vec3Origin;
extern vec4_t	vec4Origin;
extern mat4x4_t	mat4x4Identity;
extern mat3x3_t	axisIdentity;
extern quat_t	quatIdentity;

// ===========================================================================

#ifndef M_PI
# define M_PI			3.14159265358979323846f		// matches value in gcc v2 math.h
#endif

// angle indexes
enum
{
	PITCH,		// up / down
	YAW,		// left / right
	ROLL		// fall over
};

inline float DEG2RAD(float v) { return (v * (M_PI / 180.0f)); }
inline float RAD2DEG(float v) { return (v * (180.0f / M_PI)); }

inline short ANGLE2SHORT(float x) { return ((int)(x*65536/360) & 65535); }
inline float SHORT2ANGLE(short x) { return (x*(360.0f/65536)); }

inline short ANGLE2SHORT_COMPRESS(float x) { return ((short)(x * (float)SHRT_MAX)); }
inline float SHORT2ANGLE_COMPRESS(short x) { return ((float)x/(float)SHRT_MAX); }

inline byte ANGLE2BYTE(float x) { return ((int)(x*256/360) & 255); }
inline float BYTE2ANGLE(byte x) { return (x*(360.0f/256)); }

#include "Vector.h"

// ===========================================================================

#define LARGE_EPSILON 0.1
#define SMALL_EPSILON 0.01
#define TINY_EPSILON 0.001

#define TINY_NUMBER (1.e-8)
#define SMALL_NUMBER (1.e-4)

// ===========================================================================

#define NUMVERTEXNORMALS	162
extern vec3_t	m_byteDirs[NUMVERTEXNORMALS];

byte		DirToByte(const vec3_t dirVec);
void		ByteToDir(const byte dirByte, vec3_t dirVec);

// ===========================================================================

byte		FloatToByte(float x);

float		ColorNormalizef(const float *in, float *out);
float		ColorNormalizeb(const float *in, byte *out);

// ===========================================================================

#define Vec2Add(a,b,out)		((out)[0]=(a)[0]+(b)[0],(out)[1]=(a)[1]+(b)[1])
#define Vec2Clear(in)			(*(int *)&(in)[0]=0,*(int *)&(in)[1]=0)
#define Vec2Compare(v1,v2)		((v1)[0]==(v2)[0] && (v1)[1]==(v2)[1])
#define Vec2Copy(a,b)			((b)[0]=(a)[0],(b)[1]=(a)[1])
#define Vec2DistSquared(v1,v2)	(((v1)[0]-(v2)[0])*((v1)[0]-(v2)[0])+((v1)[1]-(v2)[1])*((v1)[1]-(v2)[1]))
#define Vec2Dist(v1,v2)			(sqrtf(Vec2DistSquared((v1),(v2))))
#define Vec2DistFast(v1,v2)		(Q_FastSqrt(Vec2DistSquared((v1),(v2))))
#define Vec2Inverse(v)			((v)[0]=-(v)[0],(v)[1]=-(v)[1])
#define Vec2MA(v,s,b,o)			((o)[0]=(v)[0]+(b)[0]*(s),(o)[1]=(v)[1]+(b)[1]*(s))
#define Vec2Negate(a,b)			((b)[0]=-(a)[0],(b)[1]=-(a)[1])
#define Vec2Scale(in,s,out)		((out)[0]=(in)[0]*(s),(out)[1]=(in)[1]*(s))
#define Vec2Set(v,x,y)			((v)[0]=(x),(v)[1]=(y))
#define Vec2Subtract(a,b,c)		((c)[0]=(a)[0]-(b)[0],(c)[1]=(a)[1]-(b)[1])

#define CrossProduct(v1,v2,cr)	((cr)[0]=(v1)[1]*(v2)[2]-(v1)[2]*(v2)[1],(cr)[1]=(v1)[2]*(v2)[0]-(v1)[0]*(v2)[2],(cr)[2]=(v1)[0]*(v2)[1]-(v1)[1]*(v2)[0])
#define DotProduct(x,y)			((x)[0]*(y)[0]+(x)[1]*(y)[1]+(x)[2]*(y)[2])

#define Vec3Add(a,b,out)		((out)[0]=(a)[0]+(b)[0],(out)[1]=(a)[1]+(b)[1],(out)[2]=(a)[2]+(b)[2])
#define Vec3Average(a,b,c)		((c)[0]=((a)[0]+(b)[0])*0.5f,(c)[1]=((a)[1]+(b)[1])*0.5f, (c)[2]=((a)[2]+(b)[2])*0.5f)
#define Vec3Clear(a)			(*(int *)&(a)[0]=0,*(int *)&(a)[1]=0,*(int *)&(a)[2]=0)
#define Vec3Compare(v1,v2)		((v1)[0]==(v2)[0] && (v1)[1]==(v2)[1] && (v1)[2]==(v2)[2])
#define Vec3Copy(a,b)			((b)[0]=(a)[0],(b)[1]=(a)[1],(b)[2]=(a)[2])
#define Vec3DistSquared(v1,v2)	((((v1)[0]-(v2)[0])*((v1)[0]-(v2)[0])+((v1)[1]-(v2)[1])*((v1)[1]-(v2)[1])+((v1)[2]-(v2)[2])*((v1)[2]-(v2)[2])))
#define Vec3Dist(v1,v2)			(sqrtf(Vec3DistSquared((v1),(v2))))
#define Vec3DistFast(v1,v2)		(Q_FastSqrt(Vec3DistSquared((v1),(v2))))
#define Vec3Inverse(v)			((v)[0]=-(v)[0],(v)[1]=-(v)[1],(v)[2]=-(v)[2])
#define Vec3Length(v)			(sqrtf(DotProduct((v),(v))))
#define Vec3LengthFast(v)		(Q_FastSqrt(DotProduct((v),(v))))
#define Vec3MA(v,s,b,o)			((o)[0]=(v)[0]+(b)[0]*(s),(o)[1]=(v)[1]+(b)[1]*(s),(o)[2]=(v)[2]+(b)[2]*(s))
#define Vec3Negate(a,b)			((b)[0]=-(a)[0],(b)[1]=-(a)[1],(b)[2]=-(a)[2])
#define Vec3Scale(in,s,out)		((out)[0]=(in)[0]*(s),(out)[1]=(in)[1]*(s),(out)[2]=(in)[2]*(s))
#define Vec3Set(v,x,y,z)		((v)[0]=(x),(v)[1]=(y),(v)[2]=(z))
#define Vec3Subtract(a,b,c)		((c)[0]=(a)[0]-(b)[0],(c)[1]=(a)[1]-(b)[1],(c)[2]=(a)[2]-(b)[2])

#define Vec4Add(a,b,c)			((c)[0]=(a)[0]+(b)[0],(c)[1]=(a)[1]+(b)[1],(c)[2]=(a)[2]+(b)[2],(c)[3]=(a)[3]+(b)[3])
#define Vec4Average(a,b,c)		((c)[0]=((a)[0]+(b)[0])*0.5f,(c)[1]=((a)[1]+(b)[1])*0.5f,(c)[2]=((a)[2]+(b)[2])*0.5f,(c)[3]=((a)[3]+(b)[3])*0.5f)
#define Vec4Clear(a)			(*(int *)&(a)[0]=0,*(int *)&(a)[1]=0,*(int *)&(a)[2]=0,*(int *)&(a)[3]=0)
#define Vec4Copy(a,b)			((b)[0]=(a)[0],(b)[1]=(a)[1],(b)[2]=(a)[2],(b)[3]=(a)[3])
#define Vec4Identity(a)			((a)[0]=(a)[1]=(a)[2]=(a)[3]=0)
#define Vec4MA(v,s,b,o)			((o)[0]=(v)[0]+(b)[0]*(s),(o)[1]=(v)[1]+(b)[1]*(s),(o)[2]=(v)[2]+(b)[2]*(s),(o)[3]=(v)[3]+(b)[3]*(s))
#define Vec4Negate(a,b)			((b)[0]=-(a)[0],(b)[1]=-(a)[1],(b)[2]=-(a)[2],(b)[3]=-(a)[3])
#define Vec4Scale(in,s,out)		((out)[0]=(in)[0]*(s),(out)[1]=(in)[1]*(s),(out)[2]=(in)[2]*(s),(out)[3]=(in)[3]*(s))
#define Vec4Set(v,u,x,y,z)		((v)[0]=(u),(v)[1]=(x),(v)[2]=(y),(v)[3]=(z))
#define Vec4Subtract(a,b,c)		((c)[0]=(a)[0]-(b)[0],(c)[1]=(a)[1]-(b)[1],(c)[2]=(a)[2]-(b)[2],(c)[3]=(a)[3]-(b)[3])

// ===========================================================================

#define Q_rint(x)	((x) < 0 ? ((int)((x)-0.5f)) : ((int)((x)+0.5f)))

#ifdef id386
long	Q_ftol (float f);
float	Q_FastSqrt (float value);
#else // id386
# define Q_ftol(f) ((long)(f))
# define Q_FastSqrt(v) (sqrt(v))
#endif // id386

inline void Q_SinCosf(const float X, float *Sin, float *Cos)
{
#if defined(HAVE_SINCOSF)
	sincosf(X, &Sin, &Cos);
#elif defined(id386)
	__asm fld X
	__asm fsincos
	__asm mov ecx, Cos
	__asm mov edx, Sin
	__asm fstp dword ptr [ecx]
	__asm fstp dword ptr [edx]
#else
	Sin = sinf(X);
	Cos = cosf(X);
#endif
}

float	Q_RSqrtf (float number);
double	Q_RSqrtd (double number);
int		Q_log2 (int val);

template<typename TType>
inline TType Q_NearestPow(const TType Value, const bool bRoundDown)
{
	TType Result;
	for (Result=1 ; Result<Value ; Result<<=1) ;

	// If it
	if (bRoundDown && Result > Value)
		Result >>= 1;

	return Result;
}

float	Q_CalcFovY (float fovX, float width, float height);

// ===========================================================================

void		NormToLatLong(const vec3_t normal, byte out[2]);
void		LatLongToNorm(const byte latlong[2], vec3_t out);
void		MakeNormalVectorsf (const vec3_t forward, vec3_t right, vec3_t up);
void		PerpendicularVector (const vec3_t src, vec3_t dst);
void		RotatePointAroundVector(vec3_t dest, const vec3_t dir, const vec3_t point, const float degrees);
float		VectorNormalizef (const vec3_t in, vec3_t out);
float		VectorNormalizeFastf (vec3_t v);

//
// m_angles.c
//
float		AngleModf (float a);
void		Angles_Matrix3 (vec3_t angles, mat3x3_t axis);
void		Angles_Vectors (vec3_t angles, vec3_t forward, vec3_t right, vec3_t up);
float		LerpAngle (float a1, float a2, float frac);
void		VecToAngles (vec3_t vec, vec3_t angles);
void		VecToAngleRolled (vec3_t value, float angleYaw, vec3_t angles);
float		VecToYaw (vec3_t vec);

//
// m_bounds.c
//
void		AddBoundsTo2DBounds (vec2_t inMins, vec2_t inMaxs, vec2_t outMins, vec2_t outMaxs);
void		AddPointTo2DBounds (vec2_t v, vec2_t mins, vec2_t maxs);
void		Clear2DBounds (vec2_t mins, vec2_t maxs);

void		AddPointToBounds (vec3_t v, vec3_t mins, vec3_t maxs);
bool		BoundsAndSphereIntersect (const vec3_t mins, const vec3_t maxs, const vec3_t centre, float radius);
bool		BoundsIntersect (const vec3_t mins1, const vec3_t maxs1, const vec3_t mins2, const vec3_t maxs2);
void		ClearBounds (vec3_t mins, vec3_t maxs);
void		MinMins (vec3_t a, vec3_t b, vec3_t out);
void		MaxMaxs (vec3_t a, vec3_t b, vec3_t out);
float		RadiusFromBounds (vec3_t mins, vec3_t maxs);

//
// m_mat3.c
//
void		Matrix3_Angles (mat3x3_t mat, vec3_t angles);
bool		Matrix3_Compare (mat3x3_t a, mat3x3_t b);
void		Matrix3_Copy (mat3x3_t in, mat3x3_t out);
void		Matrix3_FromPoints (vec3_t v1, vec3_t v2, vec3_t v3, mat3x3_t m);
void		Matrix3_Identity (mat3x3_t mat);
void		Matrix3_Matrix4 (mat3x3_t in, vec3_t origin, mat4x4_t out);
void		Matrix3_Multiply (mat3x3_t in1, mat3x3_t in2, mat3x3_t out);
void		Matrix3_Quat (mat3x3_t m, quat_t q);
void		Matrix3_Rotate (mat3x3_t a, float angle, float x, float y, float z);
void		Matrix3_TransformVector(const mat3x3_t m, const vec3_t v, vec3_t out);
void		Matrix3_Transpose (mat3x3_t in, mat3x3_t out);

//
// m_mat4.c
//
bool		Matrix4_Compare(const mat4x4_t a, const mat4x4_t b);
void		Matrix4_Copy(const mat4x4_t a, mat4x4_t b);
void		Matrix4_Copy2D(const mat4x4_t m1, mat4x4_t m2);
void		Matrix4_Identity(mat4x4_t mat);
void		Matrix4_Matrix3(const mat4x4_t in, mat3x3_t out);
void		Matrix4_Multiply(const mat4x4_t a, const mat4x4_t b, mat4x4_t product);
void		Matrix4_Multiply2D(const mat4x4_t m1, const mat4x4_t m2, mat4x4_t out);
void		Matrix4_Multiply_Vec3(const mat4x4_t m, const vec3_t v, vec3_t out);
void		Matrix4_Multiply_Vec4(const mat4x4_t m, const vec4_t v, vec4_t out);
void		Matrix4_MultiplyFast(const mat4x4_t a, const mat4x4_t b, mat4x4_t product);
void		Matrix4_MultiplyFast2(const mat4x4_t m1, const mat4x4_t m2, mat4x4_t out);
void		Matrix4_Rotate(mat4x4_t a, const float angle, const float x, const float y, const float z);
void		Matrix4_Scale(mat4x4_t m, const float x, const float y, const float z);
void		Matrix4_Scale2D(mat4x4_t m, const float x, const float y);
void		Matrix4_Stretch2D(mat4x4_t m, const float s, const float t);
void		Matrix4_Translate(mat4x4_t m, const float x, const float y, const float z);
void		Matrix4_Translate2D(mat4x4_t m, const float x, const float y);
void		Matrix4_Transpose(const mat4x4_t m, mat4x4_t ret);

//
// m_quat.c
//
void		Quat_ConcatTransforms (quat_t q1, vec3_t v1, quat_t q2, vec3_t v2, quat_t q, vec3_t v);
void		Quat_Copy (quat_t q1, quat_t q2);
void		Quat_Conjugate (quat_t q1, quat_t q2);
void		Quat_Identity (quat_t q);
float		Quat_Inverse (quat_t q1, quat_t q2);
float		Quat_Normalize (quat_t q);
void		Quat_Lerp (quat_t q1, quat_t q2, float t, quat_t out);
void		Quat_Matrix3 (quat_t q, mat3x3_t m);
void		Quat_Multiply (quat_t q1, quat_t q2, quat_t out);
void		Quat_TransformVector (quat_t q, vec3_t v, vec3_t out);

// ===========================================================================

struct QuadVertices
{
	vec2_t verts[4];
	vec2_t stCoords[4];

	QuadVertices()
	{
		SetCoords(0, 0, 1, 1);
	}

	QuadVertices (const QuadVertices &r)
	{
		memcpy(verts, r.verts, sizeof(verts));
		memcpy(stCoords, r.stCoords, sizeof(stCoords));
	}

	QuadVertices &SetVertices (float x, float y, int width, int height)
	{
		return SetVertices(x, y, x + width, y, x + width, y + height, x, y + height);
	}

	QuadVertices &SetVertices(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4)
	{
		Vec2Set(verts[0], x1, y1);
		Vec2Set(verts[1], x2, y2);
		Vec2Set(verts[2], x3, y3);
		Vec2Set(verts[3], x4, y4);

		return *this;
	}
	
	QuadVertices &SetCoords (float s1, float t1, float s2, float t2)
	{
		return SetCoords(s1, t1, s2, t1, s2, t2, s1, t2);
	}

	QuadVertices &SetCoords(float s1, float t1, float s2, float t2, float s3, float t3, float s4, float t4)
	{
		Vec2Set(stCoords[0], s1, t1);
		Vec2Set(stCoords[1], s2, t2);
		Vec2Set(stCoords[2], s3, t3);
		Vec2Set(stCoords[3], s4, t4);
		 
		return *this;
	}
};