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

// game.h
// - game dll information visible to server

#define GAME_APIVERSION		3

// edict->svFlags

#define SVF_NOCLIENT			BIT(0)	// don't send entity to clients, even if it has effects
#define SVF_DEADMONSTER			BIT(1) // treat as CONTENTS_DEADMONSTER for collision
#define SVF_MONSTER				BIT(2)	// treat as CONTENTS_MONSTER for collision
#define SVF_EVENT				BIT(3) // send only required event values

// edict->solid values
typedef enum {
	SOLID_NOT,			// no interaction with other objects
	SOLID_TRIGGER,		// only touch when inside, after moving
	SOLID_BBOX,			// touch on edge
	SOLID_BSP			// bsp clip, touch on edge
} solid_t;

// ==========================================================================

// link_t is only used for entity area links now
struct link_t {
	link_t	*prev, *next;
};

#define MAX_ENT_CLUSTERS	16

struct
#ifndef GAME_INCLUDE
	gclient_t
#else
	gclientinternal_t
#endif
{
	playerState_t		playerState;		// communicated by server to clients
	int					ping;
	// the game dll can add anything it wants after
	// this point in the structure
};

struct
#ifndef GAME_INCLUDE
	edict_t
#else
	edictinternal_t
#endif
{
	entityState_t		s;
#ifndef GAME_INCLUDE
	gclient_t
#else
	gclientinternal_t
#endif
						*client;	// NULL if not a player
									// the server expects the first part
									// of gclient_s to be a player_state_t
									// but the rest of it is opaque
	BOOL				inUse;
	int					linkCount;

	// FIXME: move these fields to a server private sv_entity_t
	link_t				area;				// linked to a division node or leaf
	
	int					numClusters;		// if -1, use headnode instead
	int					clusterNums[MAX_ENT_CLUSTERS];
	int					headNode;			// unused if numClusters != -1
	int					areaNum, areaNum2;

	//================================

	int					svFlags;			// SVF_NOCLIENT, SVF_DEADMONSTER, SVF_MONSTER, etc
	vec3_t				mins, maxs;
	vec3_t				absMin, absMax, size;
	solid_t				solid;
	int					clipMask;

#ifndef GAME_INCLUDE
	edict_t
#else
	edictinternal_t
#endif
						*owner;

	// the game dll can add anything it wants after
	// this point in the structure
};

#define DEFAULT_BULLET_HSPREAD	300
#define DEFAULT_BULLET_VSPREAD	500
#define DEFAULT_SHOTGUN_HSPREAD	1000
#define DEFAULT_SHOTGUN_VSPREAD	500
#define DEFAULT_DEATHMATCH_SHOTGUN_COUNT	12
#define DEFAULT_SHOTGUN_COUNT	12
#define DEFAULT_SSHOTGUN_COUNT	20

#ifdef GAME_INCLUDE
struct edict_t;
struct gclient_t;
#endif

// ==========================================================================

//
// functions provided by the main engine
//
struct gameImport_t {
	// special messages
	void	(*bprintf) (EGamePrintLevel printlevel, char *fmt, ...);
	void	(*dprintf) (char *fmt, ...);
	void	(*cprintf) (edict_t *ent, EGamePrintLevel printLevel, char *fmt, ...);
	void	(*centerprintf) (edict_t *ent, char *fmt, ...);
	void	(*sound) (edict_t *ent, int channel, int soundIndex, float volume, float attenuation, float timeOffset);
	void	(*positioned_sound) (vec3_t origin, edict_t *ent, int channel, int soundIndex, float volume, float attenuation, float timeOffset);

	// config strings hold all the index strings, the lightstyles,
	// and misc data like the sky definition and cdtrack.
	// All of the current configstrings are sent to clients when
	// they connect, and changes are sent to all connected clients.
	void	(*configstring) (int num, char *string);

	void	(*error) (char *fmt, ...);

	// the *index functions create configstrings and some internal server state
	int		(*modelindex) (char *name);
	int		(*soundindex) (char *name);
	int		(*imageindex) (char *name);

	void (*R_GetBModelVertices) (int index, void (*receiveVertice) (TList<class btVector3> vertices));
	void (*R_GetModelVertices)(const char *model, void (*receiveVertice) (vec3_t *vertice, int *indices, int num_indices));
	void (*SV_SetPhysics) (void *world);
	class btVector3 (*R_GetBModelOrigin) (int index);
	int (*Sys_Milliseconds)();

	void	(*setmodel) (edict_t *ent, char *name);

	// collision detection
	cmTrace_t	(*trace) (vec3_t start, vec3_t mins, vec3_t maxs, vec3_t end, edict_t *passEnt, int contentMask);
	int			(*pointcontents) (vec3_t point);
	BOOL		(*inPVS) (vec3_t p1, vec3_t p2);
	BOOL		(*inPHS) (vec3_t p1, vec3_t p2);
	void		(*SetAreaPortalState) (int portalNum, BOOL open);
	BOOL		(*AreasConnected) (int area1, int area2);

	// an entity will never be sent to a client or used for collision
	// if it is not passed to linkentity.  If the size, position, or
	// solidity changes, it must be relinked.
	void	(*linkentity) (edict_t *ent);
	void	(*unlinkentity) (edict_t *ent);		// call before removing an interactive edict
	TList<edict_t*>		(*BoxEdicts) (vec3_t mins, vec3_t maxs, int areaType);
	void	(*Pmove) (pMove_t *pMove);		// player movement code common with client prediction

	// network messaging
	void	(*multicast) (vec3_t origin, EMultiCast to);
	void	(*unicast) (edict_t *ent, BOOL reliable);
	void	(*WriteChar) (int c);
	void	(*WriteByte) (int c);
	void	(*WriteShort) (int c);
	void	(*WriteLong) (int c);
	void	(*WriteFloat) (float f);
	void	(*WriteString) (char *s);
	void	(*WritePosition) (vec3_t pos);	// some fractional bits
	void	(*WriteDir) (vec3_t pos);		// single byte encoded, very coarse
	void	(*WriteAngle) (float f);

	// managed memory allocation
	void	*(*TagMalloc) (int size, int tag);
	void	(*TagFree) (void *block);
	void	(*FreeTags) (int tag);

	// console variable interaction
	cVar_t	*(*cvar) (char *varName, char *value, int flags);
	cVar_t	*(*cvar_set) (char *varName, char *value);
	cVar_t	*(*cvar_forceset) (char *varName, char *value);

	// ClientCommand and ServerCommand parameter access
	int		(*argc) ();
	char	*(*argv) (int n);
	char	*(*args) ();	// concatenation of all argv >= 1

	// add commands to the server console as if they were typed in
	// for map changing, etc
	void	(*AddCommandString) (char *text);

	void	(*DebugGraph) (float value, int color);
};

//
// functions exported by the game subsystem
//
struct gameExport_t {
	int			apiVersion;

	// the init function will only be called when a game starts,
	// not each time a level is loaded.  Persistant data for clients
	// and the server can be allocated in init
	void		(*Init) ();
	void		(*Shutdown) ();

	// each new level entered will cause a call to SpawnEntities
	void		(*SpawnEntities) (char *mapName, char *entString, char *spawnPoint);

	// Read/Write Game is for storing persistant cross level information
	// about the world state and the clients.
	// WriteGame is called every time a level is exited.
	// ReadGame is called on a loadgame.
	void		(*WriteGame) (char *fileName, BOOL autoSave);
	void		(*ReadGame) (char *fileName);

	// ReadLevel is called after the default map information has been
	// loaded with SpawnEntities
	void		(*WriteLevel) (char *filename);
	void		(*ReadLevel) (char *filename);

	BOOL		(*ClientConnect) (edict_t *ent, char *userInfo);
	void		(*ClientBegin) (edict_t *ent);
	void		(*ClientUserinfoChanged) (edict_t *ent, char *userInfo);
	void		(*ClientDisconnect) (edict_t *ent);
	void		(*ClientCommand) (edict_t *ent);
	void		(*ClientThink) (edict_t *ent, userCmd_t *cmd);

	void		(*RunFrame) ();

	// ServerCommand will be called when an "sv <command>" command is issued on the
	// server console.
	// The game can issue gi.argc() / gi.argv() commands to get the rest
	// of the parameters
	void		(*ServerCommand) ();

	//
	// global variables shared between game and server
	//

	// The edict array is allocated in the game dll so it
	// can vary in size from one game to another.
	// 
	// The size will be fixed when ge->Init() is called
	struct edict_t	*edicts;
	int				edictSize;
	int				numEdicts;		// current number, <= MAX_CS_EDICTS
	int				maxEdicts;
};

gameExport_t *GetGameApi (gameImport_t *import);

