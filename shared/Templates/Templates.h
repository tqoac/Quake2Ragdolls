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
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
*/

// Templates.h

#include "TTypes.h"
#include "TList.h"	

/*
==============================================================================

	Template functions
 
==============================================================================
*/
template<typename TType>
static inline TType Align(const TType Number, const TType Alignment)
{
	return (TType)(((intptr)Number + Alignment-1) & ~(Alignment-1));
}

#ifndef min
# define min(a,b)    (((a) < (b)) ? (a) : (b))
#endif

template<typename TType>
static inline TType Min(const TType A, const TType B)
{
	return (A<=B) ? A : B;
}
template<>
static inline float Min(const float A, const float B)
{
	return (A<=B) ? A : B;
}
template<>
static inline int Min(const int A, const int B)
{
	return (A<=B) ? A : B;
}

#ifndef max
# define max(a,b)    (((a) > (b)) ? (a) : (b))
#endif

template<typename TType>
static inline TType Max(const TType A, const TType B)
{
	return (A>=B) ? A : B;
}
template<>
static inline float Max(const float A, const float B)
{
	return (A>=B) ? A : B;
}
template<>
static inline int Max(const int A, const int B)
{
	return (A>=B) ? A : B;
}

#define bound(a,b,c)	((a) >= (c) ? (a) : (b) < (a) ? (a) : (b) > (c) ? (c) : (b))
#define clamp(a,b,c)	((b) >= (c) ? (b) : (a) < (b) ? (b) : (a) > (c) ? (c) : (a))
#define clampl(a,b,c)	((b) >= (c) ? (a)=(b) : (a) < (b) ? (a)=(b) : (a) > (c) ? (a)=(c) : (a)=(a))

template<typename TType>
static inline TType Clamp(const TType V, const TType L, const TType H)
{
	return (V<L) ? L : (V>H) ? H : V;
}
template<>
static inline float Clamp(const float V, const float L, const float H)
{
	return (V<L) ? L : (V>H) ? H : V;
}
template<>
static inline int Clamp(const int V, const int L, const int H)
{
	return (V<L) ? L : (V>H) ? H : V;
}

template<typename TType>
static inline bool IsPowOfTwo(const TType Value)
{
	return (bool)(Value > 0 && (Value & (Value-1)) == 0);
}
template<>
static inline bool IsPowOfTwo(const int Value)
{
	return (bool)(Value > 0 && (Value & (Value-1)) == 0);
}
template<>
static inline bool IsPowOfTwo(const uint32 Value)
{
	return (bool)(Value > 0 && (Value & (Value-1)) == 0);
}

#ifndef ArrayCount
# define ArrayCount(a) (sizeof(a)/sizeof((a)[0]))
#endif
