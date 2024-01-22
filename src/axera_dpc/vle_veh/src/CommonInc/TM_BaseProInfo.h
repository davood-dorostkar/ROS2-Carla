
#ifndef APL_BaseDriverInitList_H
#define APL_BaseDriverInitList_H

#ifdef __cplusplus
extern "C"
{
#endif

#define APL_PRJ_STRING_LENGTH  8
#define APL_INITSTRING_LENGTH  7

typedef  union
{
  struct
  {
    uint32  u_ComponentID   : 8;    
    uint32  u_MajorVersion  : 8;    
    uint32  u_MinorVersion  : 8;        
    uint32  u_PatchLevel    : 8;    
  } CfblDetail_t;
  uint32 u_CfblRaw;
} CfblVersDetail_t;

typedef  union
{
  struct
  {
    uint32  u_ComponentID   : 8;    
    uint32  u_MajorVersion  : 8;    
    uint32  u_MinorVersion  : 8;        
    uint32  u_PatchLevel    : 8;    
  } BresDetail_t;
  uint32 u_BresRaw;
} BresVersDetail_t;

typedef  union
{
  struct
  {
    uint32  u_ComponentID   : 8;    
    uint32  u_MajorVersion  : 8;    
    uint32  u_MinorVersion  : 8;        
    uint32  u_PatchLevel    : 8;    
  } UfblDetail_t;
  uint32 u_UfblRaw;
} UfblVersDetail_t;

typedef  union
{
  struct
  {
    uint32  u_ComponentID   : 8;    
    uint32  u_MajorVersion  : 8;    
    uint32  u_MinorVersion  : 8;        
    uint32  u_PatchLevel    : 8;    
  } PParDetail_t;
  uint32 u_PParRaw;
} PParVersDetail_t;

typedef struct APL_EntityVersBootloader_t
{
  
  BresVersDetail_t s_BresVers;    
  PParVersDetail_t s_PParVers;    
  UfblVersDetail_t s_UFBLVers;    
  
  CfblVersDetail_t s_CfblVers;    
} APL_EntityVersBootloader_t;

typedef union
{
  struct
  {
    uint32  u_ComponentID   : 8;    
    uint32  u_MajorVersion  : 8;    
    uint32  u_MinorVersion  : 8;        
    uint32  u_PatchLevel    : 8;    
  } ApplDetail_t;
  uint32 u_ApplRaw;
} ApplVersDetail_t;

typedef union
{
  struct
  {
    uint32  u_ComponentID   : 8;    
    uint32  u_MajorVersion  : 8;    
    uint32  u_MinorVersion  : 8;        
    uint32  u_PatchLevel    : 8;    
  } ASICDetail_t;
  uint32 u_ASICRaw;
} ASICVersDetail_t;

typedef struct t_HwCompatibleInfoBlock {
  
  uint16 ui16_HwCompatId;		
  
  uint16 ui16_SubCompatIdMin;		
  
  uint16 ui16_SubCompatIdMax;		
} t_HwCompatibleInfoBlock;

typedef struct PparBlockStructure_t_InfoBlock
{
  uint8 PparVersion;    
} PparBlockStructure_t_InfoBlock;

#ifdef __cplusplus
}
#endif

#endif
 
