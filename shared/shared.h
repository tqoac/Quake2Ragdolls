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

//
// shared.h
// Included first by ALL program modules
//

#ifndef __SHARED_H__
#define __SHARED_H__

#define _CRT_SECURE_NO_WARNINGS 1

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <cctype>

#include <vector>
#include <string>

#include "Platform.h"
#include "Templates/Templates.h"
#include "string.h"
#include "Exceptions.h"
#include "MathLib/MathLib.h"
#include "ColorVec.h"

#define EGL_HOMEPAGE "http://egl.quakedev.com/"

/*
=============================================================================

	PROTOCOL

=============================================================================
*/

#define ORIGINAL_PROTOCOL_VERSION		34

#define ENHANCED_PROTOCOL_VERSION		35
#define ENHANCED_COMPATIBILITY_NUMBER	1905

#define MINOR_VERSION_R1Q2_BASE			1903
#define MINOR_VERSION_R1Q2_UCMD_UPDATES	1904
#define	MINOR_VERSION_R1Q2_32BIT_SOLID	1905

//
// server to client
// note: ONLY add things to the bottom, to keep Quake2 compatibility
//
enum ESVCType
{
	SVC_BAD,

	//
	// these ops are known to the game dll
	//
	SVC_MUZZLEFLASH2,
	SVC_TEMP_ENTITY,
	SVC_LAYOUT,
	SVC_INVENTORY,

	//
	// the rest are private to the client and server (you can not modify their order!)
	//
	SVC_NOP,
	SVC_DISCONNECT,
	SVC_RECONNECT,
	SVC_SOUND,					// <see code>
	SVC_PRINT,					// [byte] id [string] null terminated string
	SVC_STUFFTEXT,				// [string] stuffed into client's console buffer, should be \n terminated
	SVC_SERVERDATA,				// [long] protocol ...
	SVC_CONFIGSTRING,			// [short] [string]
	SVC_SPAWNBASELINE,
	SVC_CENTERPRINT,			// [string] to put in center of the screen
	SVC_DOWNLOAD,				// [short] size [size bytes]
	SVC_PLAYERINFO,				// variable
	SVC_PACKETENTITIES,			// [...]
	SVC_DELTAPACKETENTITIES,	// [...]
	SVC_FRAME,

	SVC_ZPACKET,				// new for ENHANCED_PROTOCOL_VERSION
	SVC_ZDOWNLOAD,				// new for ENHANCED_PROTOCOL_VERSION

	SVC_MAX
};

//
// game print flags
//
enum EGamePrintLevel
{
	PRINT_LOW,				// pickup messages
	PRINT_MEDIUM,			// death messages
	PRINT_HIGH,				// critical messages
	PRINT_CHAT				// chat messages
};

//
// destination class for gi.multicast()
//
enum EMultiCast
{
	MULTICAST_ALL,
	MULTICAST_PHS,
	MULTICAST_PVS,
	MULTICAST_ALL_R,
	MULTICAST_PHS_R,
	MULTICAST_PVS_R
};

//
// client connection state
//
enum EClientState
{
	CA_UNINITIALIZED,	// initial state
	CA_DISCONNECTED,	// not talking to a server
	CA_CONNECTING,		// sending request packets to the server
	CA_CONNECTED,		// netChan_t established, waiting for svc_serverdata
	CA_ACTIVE			// game views should be displayed
};

//
// server state
//
enum EServerState
{
	SS_DEAD,			// no map loaded
	SS_LOADING,			// spawning level edicts
	SS_GAME,			// actively running
	SS_CINEMATIC,		// playing a cinematic
	SS_DEMO,			// playing a demo
	SS_PIC				// just showing a pic
};

// ===========================================================================

#include <random>

class MTwister
{
	std::tr1::mt19937 mt;

public:
	MTwister ();
	MTwister (uint32 seed);

	void	Seed (uint32 seed);
	uint32	Rand ();

	float Random ()
	{
		return Rand() * 0.00000000023283064365386962890625f;
	}

	float CRandom()
	{
		return ((int)Rand() - 0x7FFFFFFF) * 0.000000000465661287307739257812f;
	}

private:
	uint32 Reload();
};

extern MTwister twister;

#define rand() (twister.Rand())
#define frand() (twister.Random())	// 0 to 1
#define crand() (twister.CRandom())	// -1 to 1
#define srand(seed) (twister.Seed((uint32)seed))
inline float brandom (float min, float max)
{
	return min+((frand()*max)-min);
}

/*
==============================================================================

	PARSING
 
==============================================================================
*/

#define MAX_STRING_CHARS	1024	// max length of a string passed to Cmd_TokenizeString
#define MAX_STRING_TOKENS	256		// max tokens resulting from Cmd_TokenizeString
#define MAX_TOKEN_CHARS		512		// max length of an individual token

#include "Path.h"

char		*Com_Parse (char **dataPtr);
void		Com_DefaultExtension (char *path, char *extension, size_t size);
void		Com_FileBase (char *in, char *out);
void		Com_FileExtension (char *path, char *out, size_t size);
void		Com_FilePath (char *path, char *out, size_t size);
void		Com_NormalizePath (char *Dest, const size_t DestSize, const char *Source);
char		*Com_SkipPath (char *pathname);
void		Com_SkipRestOfLine (char **dataPtr);
char		*Com_SkipWhiteSpace (char *dataPtr, bool *hasNewLines);
void		Com_StripExtension (char *dest, size_t size, const char *src);
void		Com_StripPadding (char *in, char *dest);

/*
==============================================================================

	STRING RELATED FUNCTIONS

==============================================================================
*/

void	Q_snprintfz(char *dest, size_t size, const char *fmt, ...);
void	Q_strcatz(char *dst, const char *src, int dstSize);
size_t	Q_strncpyz(char *dest, const char *src, size_t size);

char	*Q_strlwr(char *s);
char	*Q_strupr(char *s);

#ifdef id386
int __cdecl Q_tolower (int c);
#else // id386
#define Q_tolower(chr) (tolower((chr)))
#endif // id386
#define Q_toupper(chr) (toupper((chr)))

// ===========================================================================

int		Q_WildcardMatch (const char *filter, const char *string, int ignoreCase);
char	*Q_VarArgs (char *format, ...);

/*
==============================================================================

	INFO STRINGS

==============================================================================
*/

#define MAX_INFO_KEY		64
#define MAX_INFO_VALUE		64
#define MAX_INFO_STRING		512

void	Info_Print (char *s);
char	*Info_ValueForKey (char *s, char *key);
void	Info_RemoveKey (char *s, char *key);
void	Info_SetValueForKey (char *s, char *key, char *value);
bool	Info_Validate (char *s);

/*
==============================================================================

	BYTE ORDER FUNCTIONS
 
==============================================================================
*/

extern float (*LittleFloat) (float f);
extern int (*LittleLong) (int l);
extern sint16 (*LittleShort) (sint16 s);
extern float (*BigFloat) (float f);
extern int (*BigLong) (int l);
extern sint16 (*BigShort) (sint16 s);

void		Swap_Init ();

/*
==============================================================================

	NON-PORTABLE SYSTEM SERVICES

==============================================================================
*/

typedef uint32				fileHandle_t;

#define MAX_QEXT			16		// max length of a quake game pathname extension
#define MAX_QPATH			64		// max length of a quake game pathname
#define MAX_OSPATH			128		// max length of a filesystem pathname

// directory searching
#define SFF_ARCH	0x01
#define SFF_HIDDEN	0x02
#define SFF_RDONLY	0x04
#define SFF_SUBDIR	0x08
#define SFF_SYSTEM	0x10

// these are used for FS_OpenFile
enum EFSOpenMode
{
	FS_MODE_READ_BINARY,
	FS_MODE_WRITE_BINARY,
	FS_MODE_APPEND_BINARY,
	FS_MODE_READ_WRITE_BINARY,

	FS_MODE_WRITE_TEXT,
	FS_MODE_APPEND_TEXT
};

// these are used for FS_Seek
enum EFSSeekOrigin
{
	FS_SEEK_SET,
	FS_SEEK_CUR,
	FS_SEEK_END
};

//
// this is only here so the functions in shared/ can link
//
#define MAX_COMPRINT 4096

// Com_Printf
typedef int comPrint_t;
enum
{
	PRNT_WARNING			= BIT(0),
	PRNT_ERROR				= BIT(1),
	PRNT_CONSOLE			= BIT(2),
	PRNT_CHATHUD			= BIT(3)
};
void	Com_Printf (comPrint_t flags, char *fmt, ...);
void	Com_DevPrintf (comPrint_t flags, char *fmt, ...);

// Com_Error
enum EComErrorType
{
	ERR_FATAL,				// exit the entire game with a popup window
	ERR_DROP,				// print to console and disconnect from game
	ERR_DISCONNECT			// don't kill server
};
void	Com_Error (EComErrorType code, char *fmt, ...);

//
// styles for R_DrawString/Char
//
enum
{
	FS_ALIGN_CENTER			= BIT(0),
	FS_ALIGN_RIGHT			= BIT(1),
	FS_ITALIC				= BIT(2),
	FS_SECONDARY			= BIT(3),
	FS_SHADOW				= BIT(4),
	FS_SQUARE				= BIT(5),	// Force the width/height to the character width/height value that's largest
};

/*
==============================================================================

	CVARS

	Console variables
	Do NOT modify struct fields, use the functions
==============================================================================
*/

enum
{
	CVAR_ARCHIVE		= BIT(0),	// saved to config
	CVAR_USERINFO		= BIT(1),	// added to userinfo  when changed
	CVAR_SERVERINFO		= BIT(2),	// added to serverinfo when changed
	CVAR_READONLY		= BIT(3),	// can only be changed when forced through code
	CVAR_LATCH_SERVER	= BIT(4),	// delay changes until server restart
	CVAR_LATCH_VIDEO	= BIT(5),	// delay changes until video restart
	CVAR_LATCH_AUDIO	= BIT(6),	// delay changes until audio restart
	CVAR_RESET_GAMEDIR	= BIT(7),	// reset game dir when this cvar is modified
	CVAR_CHEAT			= BIT(8),	// clamp to the default value when cheats are off
};

struct cVar_t
{
	char			*name;
	char			*string;
	char			*latchedString;	// for CVAR_LATCH vars
	int				flags;
	bool			modified;		// set each time the cvar is changed
	float			floatVal;

	int				intVal;
};


// SCRIPTING
#include "cm_scripting.h"

/*
==============================================================================

	CONSOLE COMMANDS

==============================================================================
*/

enum
{
	CMD_CGAME		= BIT(0),	// Automatically added by the engine
};

/*
==============================================================================

	CONTENTS/SURFACE FLAGS

==============================================================================
*/

//
// lower bits are stronger, and will eat weaker brushes completely
//
#define CONTENTS_SOLID			1		// an eye is never valid in a solid
#define CONTENTS_WINDOW			2		// translucent, but not watery
#define CONTENTS_AUX			4
#define CONTENTS_LAVA			8
#define CONTENTS_SLIME			16
#define CONTENTS_WATER			32
#define CONTENTS_MIST			64

// Q3BSP
#define CONTENTS_FOG			64
// !Q3BSP

//
// remaining contents are non-visible, and don't eat brushes
//
#define CONTENTS_AREAPORTAL		0x8000

#define CONTENTS_PLAYERCLIP		0x10000
#define CONTENTS_MONSTERCLIP	0x20000

//
// currents can be added to any other contents, and may be mixed
//
#define CONTENTS_CURRENT_0		0x40000
#define CONTENTS_CURRENT_90		0x80000
#define CONTENTS_CURRENT_180	0x100000
#define CONTENTS_CURRENT_270	0x200000
#define CONTENTS_CURRENT_UP		0x400000
#define CONTENTS_CURRENT_DOWN	0x800000

#define CONTENTS_ORIGIN			0x1000000	// removed before bsping an entity
#define CONTENTS_MONSTER		0x2000000	// should never be on a brush, only in game
#define CONTENTS_DEADMONSTER	0x4000000
#define CONTENTS_DETAIL			0x8000000	// brushes to be added after vis leafs
#define CONTENTS_TRANSLUCENT	0x10000000	// auto set if any surface has trans
#define CONTENTS_LADDER			0x20000000

// Q3BSP
#define Q3CNTNTS_TELEPORTER		0x40000
#define Q3CNTNTS_JUMPPAD		0x80000
#define Q3CNTNTS_CLUSTERPORTAL	0x100000
#define Q3CNTNTS_DONOTENTER		0x200000

#define Q3CNTNTS_ORIGIN			0x1000000	// removed before bsping an entity

#define Q3CNTNTS_BODY			0x2000000	// should never be on a brush, only in game
#define Q3CNTNTS_CORPSE			0x4000000
#define Q3CNTNTS_DETAIL			0x8000000	// brushes not used for the bsp
#define Q3CNTNTS_STRUCTURAL		0x10000000	// brushes used for the bsp
#define Q3CNTNTS_TRANSLUCENT	0x20000000	// don't consume surface fragments inside
#define Q3CNTNTS_TRIGGER		0x40000000
#define Q3CNTNTS_NODROP			0x80000000	// don't leave bodies or items (death fog, lava)
// !Q3BSP

//
// content masks
//
#define CONTENTS_MASK_ALL			(-1)
#define CONTENTS_MASK_SOLID			(CONTENTS_SOLID|CONTENTS_WINDOW)
#define CONTENTS_MASK_PLAYERSOLID	(CONTENTS_SOLID|CONTENTS_PLAYERCLIP|CONTENTS_WINDOW|CONTENTS_MONSTER)
#define CONTENTS_MASK_DEADSOLID		(CONTENTS_SOLID|CONTENTS_PLAYERCLIP|CONTENTS_WINDOW)
#define CONTENTS_MASK_MONSTERSOLID	(CONTENTS_SOLID|CONTENTS_MONSTERCLIP|CONTENTS_WINDOW|CONTENTS_MONSTER)
#define CONTENTS_MASK_WATER			(CONTENTS_WATER|CONTENTS_LAVA|CONTENTS_SLIME)
#define CONTENTS_MASK_OPAQUE		(CONTENTS_SOLID|CONTENTS_SLIME|CONTENTS_LAVA)
#define CONTENTS_MASK_SHOT			(CONTENTS_SOLID|CONTENTS_MONSTER|CONTENTS_WINDOW|CONTENTS_DEADMONSTER)
#define CONTENTS_MASK_CURRENT		(CONTENTS_CURRENT_0|CONTENTS_CURRENT_90|CONTENTS_CURRENT_180|CONTENTS_CURRENT_270|CONTENTS_CURRENT_UP|CONTENTS_CURRENT_DOWN)


#define SURF_TEXINFO_LIGHT		0x1		// value will hold the light strength
#define SURF_TEXINFO_SLICK		0x2		// effects game physics
#define SURF_TEXINFO_SKY		0x4		// don't draw, but add to skybox
#define SURF_TEXINFO_WARP		0x8		// turbulent water warp
#define SURF_TEXINFO_TRANS33	0x10
#define SURF_TEXINFO_TRANS66	0x20
#define SURF_TEXINFO_FLOWING	0x40	// scroll towards angle
#define SURF_TEXINFO_NODRAW		0x80	// don't bother referencing the texture

#define SURF_TEXINFO_HINT		0x100	// these aren't known to the engine I believe
#define SURF_TEXINFO_SKIP		0x200	// only the compiler uses them

// Q3BSP
#define SHREF_NODAMAGE			0x1		// never give falling damage
#define SHREF_SLICK				0x2		// effects game physics
#define SHREF_SKY				0x4		// lighting from environment map
#define SHREF_LADDER			0x8
#define SHREF_NOIMPACT			0x10	// don't make missile explosions
#define SHREF_NOMARKS			0x20	// don't leave missile marks
#define SHREF_FLESH				0x40	// make flesh sounds and effects
#define SHREF_NODRAW			0x80	// don't generate a drawsurface at all
#define SHREF_HINT				0x100	// make a primary bsp splitter
#define SHREF_SKIP				0x200	// completely ignore, allowing non-closed brushes
#define SHREF_NOLIGHTMAP		0x400	// surface doesn't need a lightmap
#define SHREF_POINTLIGHT		0x800	// generate lighting info at vertexes
#define SHREF_METALSTEPS		0x1000	// clanking footsteps
#define SHREF_NOSTEPS			0x2000	// no footstep sounds
#define SHREF_NONSOLID			0x4000	// don't collide against curves with this set
#define SHREF_LIGHTFILTER		0x8000	// act as a light filter during q3map -light
#define SHREF_ALPHASHADOW		0x10000	// do per-pixel light shadow casting in q3map
#define SHREF_NODLIGHT			0x20000	// never add dynamic lights
#define SHREF_DUST				0x40000 // leave a dust trail when walking on this surface
// !Q3BSP

//
// gi.BoxEdicts() can return a list of either solid or trigger entities
//
#define AREA_SOLID				1
#define AREA_TRIGGERS			2

/*
==============================================================================

	PLANE

==============================================================================
*/

// 0-2 are axial planes
#define PLANE_X			0
#define PLANE_Y			1
#define PLANE_Z			2
#define PLANE_NON_AXIAL	3

// 3-5 are non-axial planes snapped to the nearest
#define PLANE_ANYX		3
#define PLANE_ANYY		4
#define PLANE_ANYZ		5

struct plane_t
{
	vec3_t			normal;
	float			dist;
	byte			type;			// for fast side tests
	byte			signBits;		// signx + (signy<<1) + (signz<<1)
};

//
// m_plane.c
//
#define PlaneDiff(point,plane) (((plane)->type < 3 ? (point)[(plane)->type] : DotProduct((point), (plane)->normal)) - (plane)->dist)
#define BOX_ON_PLANE_SIDE(mins, maxs, p)			\
	(((p)->type < 3)? (								\
		((p)->dist <= (mins)[(p)->type])? 1 :		\
		(											\
			((p)->dist >= (maxs)[(p)->type])? 2 : 3	\
		)											\
	) : BoxOnPlaneSide((mins), (maxs), (p)))

int BoxOnPlaneSide(const vec3_t mins, const vec3_t maxs, const plane_t *plane);
int PlaneTypeForNormal(const vec3_t normal);
void CategorizePlane(plane_t *plane);
void PlaneFromPoints(const vec3_t verts[3], plane_t *plane);
bool ComparePlanes(const vec3_t p1normal, const float p1dist, const vec3_t p2normal, const float p2dist);
void SnapVector(vec3_t normal);
void ProjectPointOnPlane(vec3_t dst, const vec3_t point, const vec3_t normal);
int SignbitsForPlane(const plane_t *out);

/*
==============================================================================

	CMODEL

==============================================================================
*/

struct cmBspSurface_t
{
	char			name[16];
	int				flags;
	int				value;

	// Q3BSP
	int				contents;
};

// A trace is returned when a box is swept through the world
struct cmTrace_t
{
	bool			allSolid;	// if true, plane is not valid
	bool			startSolid;	// if true, the initial point was in a solid area
	float			fraction;	// time completed, 1.0 = didn't hit anything
	vec3_t			endPos;		// final position
	plane_t			plane;		// surface normal at impact
	cmBspSurface_t	*surface;	// surface hit
	int				contents;	// contents on other side of surface hit
	struct edict_t	*ent;		// not set by CM_*() functions
};

/*
==============================================================================

	PREDICTION

==============================================================================
*/

// pMoveState_t is the information necessary for client side movement prediction
enum
{
	// can accelerate and turn
	PMT_NORMAL,
	PMT_SPECTATOR,
	// no acceleration or turning
	PMT_DEAD,
	PMT_GIB,		// different bounding box
	PMT_FREEZE
};

// pmove->pmFlags
enum
{
	PMF_DUCKED			= BIT(0),
	PMF_JUMP_HELD		= BIT(1),
	PMF_ON_GROUND		= BIT(2),
	PMF_TIME_WATERJUMP	= BIT(3),	// pm_time is waterjump
	PMF_TIME_LAND		= BIT(4),	// pm_time is time before rejump
	PMF_TIME_TELEPORT	= BIT(5),	// pm_time is non-moving time
	PMF_NO_PREDICTION	= BIT(6)	// temporarily disables prediction (used for grappling hook)
};

// this structure needs to be communicated bit-accurate
// from the server to the client to guarantee that
// prediction stays in sync, so no floats are used.
// if any part of the game code modifies this struct, it
// will result in a prediction error of some degree.
struct pMoveState_t
{
	int				pmType;

	sint16			origin[3];		// 12.3
	sint16			velocity[3];	// 12.3
	byte			pmFlags;		// ducked, jump_held, etc
	byte			pmTime;			// each unit = 8 ms
	sint16			gravity;
	sint16			deltaAngles[3];	// add to command angles to get view direction
									// changed by spawns, rotating objects, and teleporters
};

//
// button bits
//
#define BUTTON_ATTACK		1
#define BUTTON_USE			2

//stolen for r1q2 in the name of bandwidth
#define	BUTTON_UCMD_DBLFORWARD	4
#define BUTTON_UCMD_DBLSIDE		8
#define	BUTTON_UCMD_DBLUP		16

#define BUTTON_UCMD_DBL_ANGLE1	32
#define BUTTON_UCMD_DBL_ANGLE2	64

#define BUTTON_ANY			128			// any key whatsoever

// userCmd_t is sent to the server each client frame
struct userCmd_t
{
	byte		msec;
	byte		buttons;

	sint16		angles[3];

	sint16		forwardMove;
	sint16		sideMove;
	sint16		upMove;

	byte		impulse;		// remove?
	byte		lightLevel;		// light level the player is standing on
};


#define MAXTOUCH	32
struct pMove_t
{
	// state (in / out)
	pMoveState_t	state;

	// command (in)
	userCmd_t		cmd;
	bool			snapInitial;	// if s has been changed outside pmove

	// results (out)
	int				numTouch;
	struct edict_t	*touchEnts[MAXTOUCH];

	vec3_t			viewAngles;			// clamped
	float			viewHeight;

	vec3_t			mins, maxs;			// bounding box size

	struct edict_t	*groundEntity;
	int				waterType;
	int				waterLevel;

	// callbacks to test the world
	cmTrace_t		(*trace) (vec3_t start, vec3_t mins, vec3_t maxs, vec3_t end);
	int				(*pointContents) (vec3_t point);
};

struct pMoveNew_t
{
	// state (in / out)
	pMoveState_t	state;

	// command (in)
	userCmd_t		cmd;
	bool			snapInitial;	// if s has been changed outside pmove

	// results (out)
	int				numTouch;
	struct edict_t	*touchEnts[MAXTOUCH];

	vec3_t			viewAngles;			// clamped
	float			viewHeight;

	vec3_t			mins, maxs;			// bounding box size

	struct edict_t	*groundEntity;
	int				waterType;
	int				waterLevel;

	// callbacks to test the world
	cmTrace_t		(*trace) (vec3_t start, vec3_t mins, vec3_t maxs, vec3_t end);
	int				(*pointContents) (vec3_t point);

	float			multiplier;
	bool			strafeHack;
	bool			step;
};

/*
==============================================================================

	ENTITY FX

==============================================================================
*/

// entityState_t->effects
// Effects are things handled on the client side (lights, particles, frame
// animations) that happen constantly on the given entity. An entity that has
// effects will be sent to the client even if it has a zero index model.
#define EF_ROTATE			0x00000001		// rotate (bonus items)
#define EF_GIB				0x00000002		// leave a trail
#define EF_BLASTER			0x00000008		// redlight + trail
#define EF_ROCKET			0x00000010		// redlight + trail
#define EF_GRENADE			0x00000020
#define EF_HYPERBLASTER		0x00000040
#define EF_BFG				0x00000080
#define EF_COLOR_SHELL		0x00000100
#define EF_POWERSCREEN		0x00000200
#define EF_ANIM01			0x00000400		// automatically cycle between frames 0 and 1 at 2 hz
#define EF_ANIM23			0x00000800		// automatically cycle between frames 2 and 3 at 2 hz
#define EF_ANIM_ALL			0x00001000		// automatically cycle through all frames at 2hz
#define EF_ANIM_ALLFAST		0x00002000		// automatically cycle through all frames at 10hz
#define EF_FLIES			0x00004000
#define EF_QUAD				0x00008000
#define EF_PENT				0x00010000
#define EF_TELEPORTER		0x00020000		// particle fountain
#define EF_FLAG1			0x00040000
#define EF_FLAG2			0x00080000

// RAFAEL
#define EF_IONRIPPER		0x00100000
#define EF_GREENGIB			0x00200000
#define EF_BLUEHYPERBLASTER 0x00400000
#define EF_SPINNINGLIGHTS	0x00800000
#define EF_PLASMA			0x01000000
#define EF_TRAP				0x02000000

// ROGUE
#define EF_TRACKER			0x04000000
#define EF_DOUBLE			0x08000000
#define EF_SPHERETRANS		0x10000000
#define EF_TAGTRAIL			0x20000000
#define EF_HALF_DAMAGE		0x40000000
#define EF_TRACKERTRAIL		0x80000000

/*
==============================================================================

	RENDERFX

==============================================================================
*/

// entityState_t->renderfx flags
#define RF_MINLIGHT			BIT(0)		// allways have some light (viewmodel)
#define RF_VIEWERMODEL		BIT(1)		// don't draw through eyes, only mirrors
#define RF_WEAPONMODEL		BIT(2)		// only draw through eyes
#define RF_FULLBRIGHT		BIT(3)		// allways draw full intensity
#define RF_DEPTHHACK		BIT(4)		// for view weapon Z crunching
#define RF_TRANSLUCENT		BIT(5)
#define RF_FRAMELERP		BIT(6)
#define RF_BEAM				BIT(7)
#define RF_CUSTOMSKIN		BIT(8)	// skin is an index in image_precache
#define RF_GLOW				BIT(9)	// pulse lighting for bonus items

#define RF_SHELL_RED		BIT(10)
#define RF_SHELL_GREEN		BIT(11)
#define RF_SHELL_BLUE		BIT(12)

#define RF_IR_VISIBLE		0x00008000		// 32768
#define RF_SHELL_DOUBLE		0x00010000		// 65536
#define RF_SHELL_HALF_DAM	0x00020000
#define RF_USE_DISGUISE		0x00040000

#define RF_NOSHADOW			0x00080000
#define RF_CULLHACK			0x00100000
#define RF_FORCENOLOD		0x00200000
#define RF_SHELLMASK		(RF_SHELL_HALF_DAM|RF_SHELL_DOUBLE|RF_SHELL_RED|RF_SHELL_GREEN|RF_SHELL_BLUE)

/*
==============================================================================

	MUZZLE FLASHES

==============================================================================
*/

// muzzle flashes / player effects
enum
{
	MZ_BLASTER,
	MZ_MACHINEGUN,
	MZ_SHOTGUN,
	MZ_CHAINGUN1,
	MZ_CHAINGUN2,
	MZ_CHAINGUN3,
	MZ_RAILGUN,
	MZ_ROCKET,
	MZ_GRENADE,
	MZ_LOGIN,
	MZ_LOGOUT,
	MZ_RESPAWN,
	MZ_BFG,
	MZ_SSHOTGUN,
	MZ_HYPERBLASTER,
	MZ_ITEMRESPAWN,

	// RAFAEL
	MZ_IONRIPPER,
	MZ_BLUEHYPERBLASTER,
	MZ_PHALANX,

	// ROGUE
	MZ_ETF_RIFLE		= 30,
	MZ_SHOTGUN2,
	MZ_HEATBEAM,
	MZ_BLASTER2,
	MZ_TRACKER,
	MZ_NUKE1,
	MZ_NUKE2,
	MZ_NUKE4,
	MZ_NUKE8,
	// ROGUE

	MZ_SILENCED			= 128,		// bit flag ORed with one of the above numbers
};

// monster muzzle flashes

extern	vec3_t dumb_and_hacky_monster_MuzzFlashOffset [];

enum
{
	MZ2_TANK_BLASTER_1				= 1,
	MZ2_TANK_BLASTER_2,
	MZ2_TANK_BLASTER_3,
	MZ2_TANK_MACHINEGUN_1,
	MZ2_TANK_MACHINEGUN_2,
	MZ2_TANK_MACHINEGUN_3,
	MZ2_TANK_MACHINEGUN_4,
	MZ2_TANK_MACHINEGUN_5,
	MZ2_TANK_MACHINEGUN_6,
	MZ2_TANK_MACHINEGUN_7,
	MZ2_TANK_MACHINEGUN_8,
	MZ2_TANK_MACHINEGUN_9,
	MZ2_TANK_MACHINEGUN_10,
	MZ2_TANK_MACHINEGUN_11,
	MZ2_TANK_MACHINEGUN_12,
	MZ2_TANK_MACHINEGUN_13,
	MZ2_TANK_MACHINEGUN_14,
	MZ2_TANK_MACHINEGUN_15,
	MZ2_TANK_MACHINEGUN_16,
	MZ2_TANK_MACHINEGUN_17,
	MZ2_TANK_MACHINEGUN_18,
	MZ2_TANK_MACHINEGUN_19,
	MZ2_TANK_ROCKET_1,
	MZ2_TANK_ROCKET_2,
	MZ2_TANK_ROCKET_3,

	MZ2_INFANTRY_MACHINEGUN_1,
	MZ2_INFANTRY_MACHINEGUN_2,
	MZ2_INFANTRY_MACHINEGUN_3,
	MZ2_INFANTRY_MACHINEGUN_4,
	MZ2_INFANTRY_MACHINEGUN_5,
	MZ2_INFANTRY_MACHINEGUN_6,
	MZ2_INFANTRY_MACHINEGUN_7,
	MZ2_INFANTRY_MACHINEGUN_8,
	MZ2_INFANTRY_MACHINEGUN_9,
	MZ2_INFANTRY_MACHINEGUN_10,
	MZ2_INFANTRY_MACHINEGUN_11,
	MZ2_INFANTRY_MACHINEGUN_12,
	MZ2_INFANTRY_MACHINEGUN_13,

	MZ2_SOLDIER_BLASTER_1,
	MZ2_SOLDIER_BLASTER_2,
	MZ2_SOLDIER_SHOTGUN_1,
	MZ2_SOLDIER_SHOTGUN_2,
	MZ2_SOLDIER_MACHINEGUN_1,
	MZ2_SOLDIER_MACHINEGUN_2,

	MZ2_GUNNER_MACHINEGUN_1,
	MZ2_GUNNER_MACHINEGUN_2,
	MZ2_GUNNER_MACHINEGUN_3,
	MZ2_GUNNER_MACHINEGUN_4,
	MZ2_GUNNER_MACHINEGUN_5,
	MZ2_GUNNER_MACHINEGUN_6,
	MZ2_GUNNER_MACHINEGUN_7,
	MZ2_GUNNER_MACHINEGUN_8,
	MZ2_GUNNER_GRENADE_1,
	MZ2_GUNNER_GRENADE_2,
	MZ2_GUNNER_GRENADE_3,
	MZ2_GUNNER_GRENADE_4,

	MZ2_CHICK_ROCKET_1,

	MZ2_FLYER_BLASTER_1,
	MZ2_FLYER_BLASTER_2,

	MZ2_MEDIC_BLASTER_1,

	MZ2_GLADIATOR_RAILGUN_1,

	MZ2_HOVER_BLASTER_1,

	MZ2_ACTOR_MACHINEGUN_1,

	MZ2_SUPERTANK_MACHINEGUN_1,
	MZ2_SUPERTANK_MACHINEGUN_2,
	MZ2_SUPERTANK_MACHINEGUN_3,
	MZ2_SUPERTANK_MACHINEGUN_4,
	MZ2_SUPERTANK_MACHINEGUN_5,
	MZ2_SUPERTANK_MACHINEGUN_6,
	MZ2_SUPERTANK_ROCKET_1,
	MZ2_SUPERTANK_ROCKET_2,
	MZ2_SUPERTANK_ROCKET_3,

	MZ2_BOSS2_MACHINEGUN_L1,
	MZ2_BOSS2_MACHINEGUN_L2,
	MZ2_BOSS2_MACHINEGUN_L3,
	MZ2_BOSS2_MACHINEGUN_L4,
	MZ2_BOSS2_MACHINEGUN_L5,
	MZ2_BOSS2_ROCKET_1,
	MZ2_BOSS2_ROCKET_2,
	MZ2_BOSS2_ROCKET_3,
	MZ2_BOSS2_ROCKET_4,

	MZ2_FLOAT_BLASTER_1,

	MZ2_SOLDIER_BLASTER_3,
	MZ2_SOLDIER_SHOTGUN_3,
	MZ2_SOLDIER_MACHINEGUN_3,
	MZ2_SOLDIER_BLASTER_4,
	MZ2_SOLDIER_SHOTGUN_4,
	MZ2_SOLDIER_MACHINEGUN_4,
	MZ2_SOLDIER_BLASTER_5,
	MZ2_SOLDIER_SHOTGUN_5,
	MZ2_SOLDIER_MACHINEGUN_5,
	MZ2_SOLDIER_BLASTER_6,
	MZ2_SOLDIER_SHOTGUN_6,
	MZ2_SOLDIER_MACHINEGUN_6,
	MZ2_SOLDIER_BLASTER_7,
	MZ2_SOLDIER_SHOTGUN_7,
	MZ2_SOLDIER_MACHINEGUN_7,
	MZ2_SOLDIER_BLASTER_8,
	MZ2_SOLDIER_SHOTGUN_8,
	MZ2_SOLDIER_MACHINEGUN_8,

	// --- Xian shit below ---
	MZ2_MAKRON_BFG,
	MZ2_MAKRON_BLASTER_1,
	MZ2_MAKRON_BLASTER_2,
	MZ2_MAKRON_BLASTER_3,
	MZ2_MAKRON_BLASTER_4,
	MZ2_MAKRON_BLASTER_5,
	MZ2_MAKRON_BLASTER_6,
	MZ2_MAKRON_BLASTER_7,
	MZ2_MAKRON_BLASTER_8,
	MZ2_MAKRON_BLASTER_9,
	MZ2_MAKRON_BLASTER_10,
	MZ2_MAKRON_BLASTER_11,
	MZ2_MAKRON_BLASTER_12,
	MZ2_MAKRON_BLASTER_13,
	MZ2_MAKRON_BLASTER_14,
	MZ2_MAKRON_BLASTER_15,
	MZ2_MAKRON_BLASTER_16,
	MZ2_MAKRON_BLASTER_17,
	MZ2_MAKRON_RAILGUN_1,
	MZ2_JORG_MACHINEGUN_L1,
	MZ2_JORG_MACHINEGUN_L2,
	MZ2_JORG_MACHINEGUN_L3,
	MZ2_JORG_MACHINEGUN_L4,
	MZ2_JORG_MACHINEGUN_L5,
	MZ2_JORG_MACHINEGUN_L6,
	MZ2_JORG_MACHINEGUN_R1,
	MZ2_JORG_MACHINEGUN_R2,
	MZ2_JORG_MACHINEGUN_R3,
	MZ2_JORG_MACHINEGUN_R4,
	MZ2_JORG_MACHINEGUN_R5,
	MZ2_JORG_MACHINEGUN_R6,
	MZ2_JORG_BFG_1,
	MZ2_BOSS2_MACHINEGUN_R1,
	MZ2_BOSS2_MACHINEGUN_R2,
	MZ2_BOSS2_MACHINEGUN_R3,
	MZ2_BOSS2_MACHINEGUN_R4,
	MZ2_BOSS2_MACHINEGUN_R5,

	// ROGUE
	MZ2_CARRIER_MACHINEGUN_L1,
	MZ2_CARRIER_MACHINEGUN_R1,
	MZ2_CARRIER_GRENADE,
	MZ2_TURRET_MACHINEGUN,
	MZ2_TURRET_ROCKET,
	MZ2_TURRET_BLASTER,
	MZ2_STALKER_BLASTER,
	MZ2_DAEDALUS_BLASTER,
	MZ2_MEDIC_BLASTER_2,
	MZ2_CARRIER_RAILGUN,
	MZ2_WIDOW_DISRUPTOR,
	MZ2_WIDOW_BLASTER,
	MZ2_WIDOW_RAIL,
	MZ2_WIDOW_PLASMABEAM,		// PMM - not used
	MZ2_CARRIER_MACHINEGUN_L2,
	MZ2_CARRIER_MACHINEGUN_R2,
	MZ2_WIDOW_RAIL_LEFT,
	MZ2_WIDOW_RAIL_RIGHT,
	MZ2_WIDOW_BLASTER_SWEEP1,
	MZ2_WIDOW_BLASTER_SWEEP2,
	MZ2_WIDOW_BLASTER_SWEEP3,
	MZ2_WIDOW_BLASTER_SWEEP4,
	MZ2_WIDOW_BLASTER_SWEEP5,
	MZ2_WIDOW_BLASTER_SWEEP6,
	MZ2_WIDOW_BLASTER_SWEEP7,
	MZ2_WIDOW_BLASTER_SWEEP8,
	MZ2_WIDOW_BLASTER_SWEEP9,
	MZ2_WIDOW_BLASTER_100,
	MZ2_WIDOW_BLASTER_90,
	MZ2_WIDOW_BLASTER_80,
	MZ2_WIDOW_BLASTER_70,
	MZ2_WIDOW_BLASTER_60,
	MZ2_WIDOW_BLASTER_50,
	MZ2_WIDOW_BLASTER_40,
	MZ2_WIDOW_BLASTER_30,
	MZ2_WIDOW_BLASTER_20,
	MZ2_WIDOW_BLASTER_10,
	MZ2_WIDOW_BLASTER_0,
	MZ2_WIDOW_BLASTER_10L,
	MZ2_WIDOW_BLASTER_20L,
	MZ2_WIDOW_BLASTER_30L,
	MZ2_WIDOW_BLASTER_40L,
	MZ2_WIDOW_BLASTER_50L,
	MZ2_WIDOW_BLASTER_60L,
	MZ2_WIDOW_BLASTER_70L,
	MZ2_WIDOW_RUN_1,
	MZ2_WIDOW_RUN_2,
	MZ2_WIDOW_RUN_3,
	MZ2_WIDOW_RUN_4,
	MZ2_WIDOW_RUN_5,
	MZ2_WIDOW_RUN_6,
	MZ2_WIDOW_RUN_7,
	MZ2_WIDOW_RUN_8,
	MZ2_CARRIER_ROCKET_1,
	MZ2_CARRIER_ROCKET_2,
	MZ2_CARRIER_ROCKET_3,
	MZ2_CARRIER_ROCKET_4,
	MZ2_WIDOW2_BEAMER_1,
	MZ2_WIDOW2_BEAMER_2,
	MZ2_WIDOW2_BEAMER_3,
	MZ2_WIDOW2_BEAMER_4,
	MZ2_WIDOW2_BEAMER_5,
	MZ2_WIDOW2_BEAM_SWEEP_1,
	MZ2_WIDOW2_BEAM_SWEEP_2,
	MZ2_WIDOW2_BEAM_SWEEP_3,
	MZ2_WIDOW2_BEAM_SWEEP_4,
	MZ2_WIDOW2_BEAM_SWEEP_5,
	MZ2_WIDOW2_BEAM_SWEEP_6,
	MZ2_WIDOW2_BEAM_SWEEP_7,
	MZ2_WIDOW2_BEAM_SWEEP_8,
	MZ2_WIDOW2_BEAM_SWEEP_9,
	MZ2_WIDOW2_BEAM_SWEEP_10,
	MZ2_WIDOW2_BEAM_SWEEP_11
	// ROGUE
};

/*
==============================================================================

	TEMP ENTITY EVENTS

==============================================================================
*/

// Temp entity events are for things that happen at a location seperate from
// any existing entity. Temporary entity messages are explicitly constructed
// and broadcast.
enum
{
	TE_GUNSHOT,
	TE_BLOOD,
	TE_BLASTER,
	TE_RAILTRAIL,
	TE_SHOTGUN,
	TE_EXPLOSION1,
	TE_EXPLOSION2,
	TE_ROCKET_EXPLOSION,
	TE_GRENADE_EXPLOSION,
	TE_SPARKS,
	TE_SPLASH,
	TE_BUBBLETRAIL,
	TE_SCREEN_SPARKS,
	TE_SHIELD_SPARKS,
	TE_BULLET_SPARKS,
	TE_LASER_SPARKS,
	TE_PARASITE_ATTACK,
	TE_ROCKET_EXPLOSION_WATER,
	TE_GRENADE_EXPLOSION_WATER,
	TE_MEDIC_CABLE_ATTACK,
	TE_BFG_EXPLOSION,
	TE_BFG_BIGEXPLOSION,
	TE_BOSSTPORT,
	TE_BFG_LASER,
	TE_GRAPPLE_CABLE,
	TE_WELDING_SPARKS,
	TE_GREENBLOOD,
	TE_BLUEHYPERBLASTER,
	TE_PLASMA_EXPLOSION,
	TE_TUNNEL_SPARKS,

	//ROGUE
	TE_BLASTER2,
	TE_RAILTRAIL2,
	TE_FLAME,
	TE_LIGHTNING,
	TE_PLAIN_EXPLOSION,
	TE_FLASHLIGHT,
	TE_FORCEWALL,
	TE_HEATBEAM,
	TE_MONSTER_HEATBEAM,
	TE_STEAM,
	TE_BUBBLETRAIL2,
	TE_MOREBLOOD,
	TE_HEATBEAM_SPARKS,
	TE_HEATBEAM_STEAM,
	TE_CHAINFIST_SMOKE,
	TE_ELECTRIC_SPARKS,
	TE_TRACKER_EXPLOSION,
	TE_TELEPORT_EFFECT,
	TE_DBALL_GOAL,
	TE_WIDOWBEAMOUT,
	TE_NUKEBLAST,
	TE_WIDOWSPLASH,
	TE_EXPLOSION1_BIG,
	TE_EXPLOSION1_NP,
	TE_FLECHETTE
	//ROGUE
};

// TE_SPLASH effects
enum
{
	SPLASH_UNKNOWN,
	SPLASH_SPARKS,
	SPLASH_BLUE_WATER,
	SPLASH_BROWN_WATER,
	SPLASH_SLIME,
	SPLASH_LAVA,
	SPLASH_BLOOD
};

/*
==============================================================================

	SOUND

==============================================================================
*/

//
// sound channels
// channel 0 never willingly overrides other channels (1-7) allways override
// a playing sound on that channel
//
enum EEntSndChannel
{
	CHAN_AUTO,
	CHAN_WEAPON,
	CHAN_VOICE,
	CHAN_ITEM,
	CHAN_BODY,

	// modifier flags
	CHAN_NO_PHS_ADD		= 8,	// send to all clients, not just ones in PHS (ATTN 0 will also do this)
	CHAN_RELIABLE		= 16	// send by reliable message, not datagram
};

//
// sound attenuation values
//
enum
{
	ATTN_NONE,				// full volume the entire level
	ATTN_NORM,
	ATTN_IDLE,
	ATTN_STATIC				// diminish very rapidly with distance
};

/*
==============================================================================

	DEATHMATCH FLAGS

==============================================================================
*/

// dmflags->floatVal flags
#define DF_NO_HEALTH		0x00000001	// 1
#define DF_NO_ITEMS			0x00000002	// 2
#define DF_WEAPONS_STAY		0x00000004	// 4
#define DF_NO_FALLING		0x00000008	// 8
#define DF_INSTANT_ITEMS	0x00000010	// 16
#define DF_SAME_LEVEL		0x00000020	// 32
#define DF_SKINTEAMS		0x00000040	// 64
#define DF_MODELTEAMS		0x00000080	// 128
#define DF_NO_FRIENDLY_FIRE	0x00000100	// 256
#define DF_SPAWN_FARTHEST	0x00000200	// 512
#define DF_FORCE_RESPAWN	0x00000400	// 1024
#define DF_NO_ARMOR			0x00000800	// 2048
#define DF_ALLOW_EXIT		0x00001000	// 4096
#define DF_INFINITE_AMMO	0x00002000	// 8192
#define DF_QUAD_DROP		0x00004000	// 16384
#define DF_FIXED_FOV		0x00008000	// 32768

#define DF_QUADFIRE_DROP	0x00010000	// 65536

#define DF_NO_MINES			0x00020000
#define DF_NO_STACK_DOUBLE	0x00040000
#define DF_NO_NUKES			0x00080000
#define DF_NO_SPHERES		0x00100000

/*
==============================================================================

	CONFIG STRINGS

==============================================================================
*/

// per-level limits
#define MAX_CS_CLIENTS		256		// absolute limit
#define MAX_CS_EDICTS		1024	// must change protocol to increase more
#define MAX_CS_LIGHTSTYLES	256
#define MAX_CS_MODELS		256		// these are sent over the net as bytes
#define MAX_CS_SOUNDS		256		// so they cannot be blindly increased
#define MAX_CS_IMAGES		256
#define MAX_CS_GENERAL		(MAX_CS_CLIENTS*2)	// general config strings

#define Q2BSP_MAX_AREAS		256
#define MAX_AREA_BITS		(Q2BSP_MAX_AREAS/8)

// config strings are a general means of communication from the server to all
// connected clients. Each config string can be at most MAX_CFGSTRLEN characters.
#define CS_NAME				0
#define CS_CDTRACK			1
#define CS_SKY				2
#define CS_SKYAXIS			3		// %f %f %f format
#define CS_SKYROTATE		4

#define CS_AIRACCEL			5		// air acceleration control
#define CS_MAXCLIENTS		6
#define CS_MAPCHECKSUM		7		// for catching cheater maps

#define CS_MODELS			8
#define CS_SOUNDS			(CS_MODELS+MAX_CS_MODELS)
#define CS_IMAGES			(CS_SOUNDS+MAX_CS_SOUNDS)
#define CS_LIGHTS			(CS_IMAGES+MAX_CS_IMAGES)
#define CS_PLAYERSKINS		(CS_LIGHTS+MAX_CS_LIGHTSTYLES)
#define CS_GENERAL			(CS_PLAYERSKINS+MAX_CS_CLIENTS)

#define MAX_CFGSTRINGS		(CS_GENERAL+MAX_CS_GENERAL)
#define MAX_CFGSTRLEN		64

// Model animations
enum refAliasAnimationType_t
{
	AA_NORMAL,
	AA_LOOP
};

struct refAnimLoopData_t
{
	int		Start;
};

class refAliasAnimation_t
{	
public:
	void					*TypeData;
	String					Name;
	int						Start;
	int						End;
	uint32					FPS;
	refAliasAnimationType_t	Type;
	int						NextAnimation;

	refAliasAnimation_t() :
	  TypeData(null),
	  Name("null"),
	  Start(0),
	  End(0),
	  FPS(0),
	  Type(AA_NORMAL),
	  NextAnimation(-1)
	{
	}

	template <typename T>
	const T *GetTypeData ()
	{
		return (T*)TypeData;
	}
};


/*
==============================================================================

	ENTITY STATE

==============================================================================
*/

// entityState_t->event values
// ertity events are for effects that take place reletive to an existing
// entities origin.  Very network efficient. All muzzle flashes really should
// be converted to events...
enum
{
	EV_NONE				= 0,
	EV_ITEM_RESPAWN,
	EV_ITEM_PICKUP,
	EV_FOOTSTEP,
	EV_FALL,			// parms: 0 = short, 1 = fall, 2 = far
	EV_TELEPORT,		// parms: 0 = player, 1 = other
	EV_MUZZLEFLASH,		// parms are the MZ_ or MZ2_ index.
	EV_EXPLOSION,		// parms: 0 = grenade, 1 water
	EV_BLASTER,
	EV_DEBUGTRAIL,
	EV_RAILTRAIL,
	EV_SPLASH,
	EV_GESTURE,
	EV_PAIN
};

// entity pain bits
enum
{
	PAINBIT_HEALTH		= 4,
	PAINBIT_ARMOR		= 8,
	PAINBIT_BLOOD		= 16,

	PAINBIT_MASK		= PAINBIT_HEALTH | PAINBIT_ARMOR | PAINBIT_BLOOD,
};

struct entityParms_t
{
private:
	byte _count;
	byte _bytes[64];
	byte _curPos;

public:
	entityParms_t() :
	  _curPos(0),
	  _count(0)
	{
	}

	entityParms_t &operator=(const entityParms_t &p)
	{
		memcpy (_bytes, p._bytes, sizeof(_bytes));
		_curPos = p._curPos;
		_count = p._count;
		return *this;
	}

	void Clear ()
	{
		_count = _curPos = 0;
	}

	byte &Count ()
	{
		return _count;
	}

	byte *Pointer ()
	{
		return _bytes;
	}

	byte Position()
	{
		return _curPos;
	}

	entityParms_t &PushByte (byte b)
	{
		_bytes[_count++] = b;
		return *this;
	}

	entityParms_t &PushChar (char b)
	{
		_bytes[_count++] = (byte)b;
		return *this;
	}

	entityParms_t &PushShort (short s)
	{
		union
		{
			short sh;
			byte b[2];
		} un;

		un.sh = s;
		memcpy (_bytes + _count, un.b, 2);
		_count += 2;
		return *this;
	}

	entityParms_t &PushInt32 (int i)
	{
		union
		{
			int in;
			byte b[4];
		} un;

		un.in = i;
		memcpy (_bytes + _count, un.b, 4);
		_count += 4;
		return *this;
	}

	entityParms_t &PushFloat (float f)
	{
		union
		{
			float fl;
			byte b[4];
		} un;

		un.fl = f;
		memcpy (_bytes + _count, un.b, 4);
		_count += 4;
		return *this;
	}

	entityParms_t &PushVector (vec3_t vec)
	{
		PushFloat(vec[0]);
		PushFloat(vec[1]);
		PushFloat(vec[2]);
		return *this;
	}

	entityParms_t &PushNormal (vec3_t vec)
	{
		PushShort(ANGLE2SHORT_COMPRESS(vec[0]));
		PushShort(ANGLE2SHORT_COMPRESS(vec[1]));
		PushShort(ANGLE2SHORT_COMPRESS(vec[2]));
		return *this;
	}

	byte ToByte (byte position)
	{
		return _bytes[position];
	}

	char ToChar (byte position)
	{
		return (char)_bytes[position];
	}

	short ToShort (byte position)
	{
		union
		{
			short sh;
			byte b[2];
		} un;

		memcpy (un.b, _bytes + position, 2);
		return un.sh;
	}

	int ToInt32 (byte position)
	{
		union
		{
			int sh;
			byte b[4];
		} un;

		memcpy (un.b, _bytes + position, 4);
		return un.sh;
	}

	float ToFloat (byte position)
	{
		union
		{
			float sh;
			byte b[4];
		} un;

		memcpy (un.b, _bytes + position, 4);
		return un.sh;
	}

	void ToVector (byte position, vec3_t pos)
	{
		pos[0] = ToFloat(position);
		pos[1] = ToFloat(position + 4);
		pos[2] = ToFloat(position + 8);
	}

	void ToNormal (byte position, vec3_t pos)
	{
		pos[0] = SHORT2ANGLE_COMPRESS(ToShort(position));
		pos[1] = SHORT2ANGLE_COMPRESS(ToShort(position + 2));
		pos[2] = SHORT2ANGLE_COMPRESS(ToShort(position + 4));
	}
};

struct entityEvent_t
{
	byte ID;
	entityParms_t parms;

	void Clear ()
	{
		ID = 0;
		parms.Clear();
	}
};

enum
{
	ET_NORMAL		= 0,	// no change in structure
	ET_QUATERNION	= 1,	// quaternion instead of euler
	ET_ITEM			= 2,	// modelIndex = index of item in itemlist
	ET_EVENT		= 4,	// event
	ET_RAGDOLL		= 8,	// ragdoll piece (modelindex encoded)
	ET_ANIMATION	= 16,	// uses animations
	ET_MISSILE		= 32,	// angles = constant velocity
};
typedef byte entityType_t;

enum
{
	RAG_HEAD,
	RAG_PELVIS,
	RAG_SPINE,
	RAG_RUPPERARM,
	RAG_RLOWERARM,
	RAG_LUPPERARM,
	RAG_LLOWERARM,
	RAG_RUPPERLEG,
	RAG_RLOWERLEG,
	RAG_LUPPERLEG,
	RAG_LLOWERLEG,

	RAG_COUNT
};

// entityState_t is the information conveyed from the server in an update
// message about entities that the client will need to render in some way
struct entityState_t
{
	entityState_t()
	{
		Clear();
	}

	void Clear(bool keepNumber = false)
	{
		if (!keepNumber)
			number = 0;
		Vec3Clear(origin);
		Vec3Clear(angles);
		memset (&events, 0, sizeof(events));
		Vec3Clear(oldOrigin);
		quat[0] = quat[1] = quat[2] = quat[3] = 0;
		modelIndex = modelIndex2 = modelIndex3 = 0;
		color = Q_BColorWhite;
		frame = 0;
		skinNum = 0;
		effects = 0;
		renderFx = solid = 0;
		sound = 0;
		type = ET_NORMAL;
		animation = -1;
	}

	// Values sent to SVF_EVENT
	ushort			number;		// edict index

	vec3_t			origin;		// entity origin or RF_BEAM start origin
	vec3_t			angles;
	entityEvent_t	events[2];	// impulse events -- muzzle flashes, footsteps, etc
								// events only go out for a single frame, they
								// are automatically cleared each frame
	vec3_t			oldOrigin;	// for interpolation or RF_BEAM end origin

	// Others
	entityType_t	type;
	quat_t			quat;

	// weapons, CTF flags, etc
	byte			modelIndex;
	byte			modelIndex2;
	byte			modelIndex3;

	colorb			color;

	ushort			frame;		// also RF_BEAM's size
	int				skinNum;	// also RF_BEAM color index

	uint32			effects;	// PGM - we're filling it, so it needs to be uint32
	int				renderFx;
	int				solid;		// for client side prediction, 8*(bits 0-4) is x/y radius
								// 8*(bits 5-9) is z down distance, 8(bits10-15) is z up
								// gi.linkentity sets this properly
	byte			sound;		// for looping sounds, to guarantee shutoff

	int				animation;
	int				oldAnimation;
};

/*
==============================================================================

	PLAYER STATE

==============================================================================
*/

// playerState->stats[] indexes
enum
{
	STAT_HEALTH,
	STAT_AMMO,
	STAT_ARMOR_ICON,
	STAT_ARMOR,
	STAT_SELECTED_ICON,
	STAT_PICKUP,
	STAT_TIMER_ICON,
	STAT_TIMER,
	STAT_HELPICON,
	STAT_SELECTED_ITEM,
	STAT_LAYOUTS,
	STAT_FRAGS,
	STAT_FLASHES,					// cleared each frame, 1 = health, 2 = armor
	STAT_CHASE,
	STAT_SPECTATOR,

	MAX_STATS				= 32
};

// playerState_t->rdFlags
enum
{
	RDF_UNDERWATER		= BIT(0),		// warp the screen as apropriate
	RDF_NOWORLDMODEL	= BIT(1),		// used for player configuration screen
	RDF_IRGOGGLES		= BIT(2),
	RDF_UVGOGGLES		= BIT(3),

	RDF_OLDAREABITS		= BIT(4),
	RDF_NOCLEAR			= BIT(5),
	RDF_FBO				= BIT(6),
};

// playerState_t is the information needed in addition to pMoveState_t to
// rendered a view.  There will only be 10 playerState_t sent each second, but
// the number of pMoveState_t changes will be relative to client frame rates.
struct playerState_t
{
	pMoveState_t	pMove;				// for prediction

	// these fields do not need to be communicated bit-precise
	vec3_t			viewAngles;			// for fixed views
	vec3_t			viewOffset;			// add to pmovestate->origin
	vec3_t			kickAngles;			// add to view direction to get render angles
										// set by weapon kicks, pain effects, etc
	int				gunAnim;

	float			viewBlend[4];		// rgba full screen effect
	
	float			fov;				// horizontal field of view

	int				rdFlags;			// refdef flags

	sint16			stats[MAX_STATS];	// fast status bar updates
};

const int ServerFrameFPS = 30;
const float ServerFrameTime = 1000.0f / ((float)ServerFrameFPS);
const float ServerFrameTimeInSeconds = ServerFrameTime / 1000.0f;
const float ServerFrameTimeInv = ServerFrameFPS / 1000.0f;

// items

// gitem_t->flags
enum
{
	IT_WEAPON		= 1,		// use makes active weapon
	IT_AMMO			= 2,
	IT_ARMOR		= 4,
	IT_STAY_COOP	= 8,
	IT_KEY			= 16,
	IT_POWERUP		= 32,
	IT_HEALTH		= 64,
	IT_GRABBABLE	= 128,
	IT_USABLE		= 256,
	IT_DROPPABLE	= 512,
};
typedef int itemFlags_t;

struct gitem_t
{
	char	*classname;	// spawning name
	char	*pickup_sound;
	char	*world_model;
	int		world_model_flags;
	char	*view_model;

	// client side info
	char	*icon;
	char	*pickup_name;	// for printing on pickup
	int		count_width;		// number of digits to display by icon

	int		quantity;		// for ammo how much, for weapons how much is used per shot
	char	*ammo;			// for weapons
	itemFlags_t	flags;			// I_* flags

	char	*weapmodel;		// weapon model index (for weapons)
	char	*precaches;		// string of all models, sounds, and images this item will use
};

struct Items
{
	enum itemIndexes
	{
		None,
		BodyArmor,
		CombatArmor,
		JacketArmor,
		ArmorShard,
		PowerScreen,
		PowerShield,
		Blaster,
		Shotgun,
		SuperShotgun,
		Machinegun,
		Chaingun,
		Grenades,
		GrenadeLauncher,
		RocketLauncher,
		HyperBlaster,
		Railgun,
		BFG,
		Shells,
		Bullets,
		Cells,
		Rockets,
		Slugs,
		QuadDamage,
		Invulnerability,
		Silencer,
		Rebreather,
		EnvironmentSuit,
		AncientHead,
		Adrenaline,
		Bandolier,
		AmmoPack,
		DataCD,
		PowerCube,
		PyramidKey,
		DataSpinner,
		SecurityPass,
		BlueKey,
		RedKey,
		CommandersHead,
		AirstrikeMarker,
		Stimpack,
		SmallHealth,
		LargeHealth,
		MegaHealth
	};

	static int numItems;
	
	static void InitWeaponRegistry();
	static int GetWeaponID (gitem_t *item);
	static gitem_t *WeaponFromID (int id);
	static int WeaponRegistryCount();
};

#define HEALTH_IGNORE_MAX	1
#define HEALTH_TIMED		2

extern	gitem_t	itemlist[];
const int MaxItems = 128;

inline const int ITEM_INDEX(gitem_t *x) {return x-itemlist;};
gitem_t	*FindItem (const char *pickup_name);
gitem_t	*FindItemByClassname (const char *classname);

#endif // __SHARED_H__
