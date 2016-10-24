#ifndef _SPRD_CAMERA_HARDWARE_CONFIG_H_
#define _SPRD_CAMERA_HARDWARE_CONFIG_H_


/* White balancing type, used for CAMERA_PARM_WHITE_BALANCING */
typedef enum
{
    CAMERA_WB_AUTO = 0,
    CAMERA_WB_INCANDESCENT,
    CAMERA_WB_FLUORESCENT = 4, //id 2 and 3 not used
    CAMERA_WB_DAYLIGHT,
    CAMERA_WB_CLOUDY_DAYLIGHT,
//    CAMERA_WB_WARM_FLUORESCENT,    //not support    
//    CAMERA_WB_TWILIGHT,	//not support
//    CAMERA_WB_SHADE,	//not support
    CAMERA_WB_MAX
} camera_wb_type;

 /* Effect type, used for CAMERA_PARM_EFFECT */
typedef enum
{
    CAMERA_EFFECT_NONE = 0, 
    CAMERA_EFFECT_MONO,
    CAMERA_EFFECT_RED,
    CAMERA_EFFECT_GREEN,
    CAMERA_EFFECT_BLUE,    
    CAMERA_EFFECT_YELLOW,
    CAMERA_EFFECT_NEGATIVE,
    CAMERA_EFFECT_SEPIA,    
//    CAMERA_EFFECT_SOLARIZE, //not support
//    CAMERA_EFFECT_POSTERIZE,	//not support
//    CAMERA_EFFECT_WHITEBOARD,	//not support
//    CAMERA_EFFECT_BLACKBOARD,	//not support
//    CAMERA_EFFECT_AQUA,	//not support
    CAMERA_EFFECT_MAX
} camera_effect_type;

 typedef enum
 {
 	CAMERA_SCENE_MODE_AUTO = 0,
	CAMERA_SCENE_MODE_NIGHT,
//	CAMERA_SCENE_MODE_ACTION, //not support
//	CAMERA_SCENE_MODE_PORTRAIT, //not support
//	CAMERA_SCENE_MODE_LANDSCAPE, //not support
//	CAMERA_SCENE_MODE_NIGHT_PORTRAIT, //not support
//	CAMERA_SCENE_MODE_THEATRE, //not support
//	CAMERA_SCENE_MODE_BEACH, //not support
//	CAMERA_SCENE_MODE_SNOW, //not support
//	CAMERA_SCENE_MODE_SUNSET, //not support
//	CAMERA_SCENE_MODE_STEADYPHOTO, //not support
//	CAMERA_SCENE_MODE_FIREWORKS, //not support
//	CAMERA_SCENE_MODE_SPORTS, //not support
//	CAMERA_SCENE_MODE_PARTY, //not support
//	CAMERA_SCENE_MODE_CANDLELIGHT, //not support
//	CAMERA_SCENE_MODE_BARCODE, //not support
	CAMERA_SCENE_MODE_MAX		
 }camera_scene_mode_type;

typedef enum{
	CAMERA_CAMERA_ID_BACK = 0,
	CAMERA_CAMERA_ID_FRONT,
	CAMERA_CAMERA_ID_MAX
}camera_id_type;

  typedef enum
 {
 	CAMERA_ZOOM_1X = 0,
	CAMERA_ZOOM_2X,
	CAMERA_ZOOM_3X,
	CAMERA_ZOOM_4X,
	CAMERA_ZOOM_MAX		
 }camera_zoom_type;

 typedef enum
{
    CAMERA_BRIGHTNESS_0 = 0,
    CAMERA_BRIGHTNESS_1 = 1,
    CAMERA_BRIGHTNESS_2 = 2,
    CAMERA_BRIGHTNESS_DEFAULT = 3,
    CAMERA_BRIGHTNESS_3 = 3,
    CAMERA_BRIGHTNESS_4 = 4,
    CAMERA_BRIGHTNESS_5 = 5,    
    CAMERA_BRIGHTNESS_6 = 6,
    CAMERA_BRIGHTNESS_MAX
} camera_brightness_type;

  typedef enum
{
    CAMERA_CONTRAST_0 = 0,
    CAMERA_CONTRAST_1 = 1,
    CAMERA_CONTRAST_2 = 2,
    CAMERA_CONTRAST_DEFAULT = 3,
    CAMERA_CONTRAST_3 = 3,
    CAMERA_CONTRAST_4 = 4,
    CAMERA_CONTRAST_5 = 5,    
    CAMERA_CONTRAST_6 = 6,
    CAMERA_CONTRAST_MAX
} camera_contrast_type;

struct str_map {
        const char *const desc;
        int val;
    };
 
 const struct str_map wb_map[] = {
        { "auto", CAMERA_WB_AUTO },
        { "incandescent", CAMERA_WB_INCANDESCENT },
        { "fluorescent", CAMERA_WB_FLUORESCENT },
        { "daylight", CAMERA_WB_DAYLIGHT },
        { "cloudy-daylight", CAMERA_WB_CLOUDY_DAYLIGHT },
        { NULL, 0 }
   };

  const struct str_map effect_map[] = {
        { "none", CAMERA_EFFECT_NONE },
        { "mono", CAMERA_EFFECT_MONO },
        { "red", CAMERA_EFFECT_RED },
        { "green", CAMERA_EFFECT_GREEN },
        { "blue", CAMERA_EFFECT_BLUE },
        //{ "yellow", CAMERA_EFFECT_YELLOW },        
        { "antique", CAMERA_EFFECT_YELLOW },        
        { "negative", CAMERA_EFFECT_NEGATIVE },
        { "sepia", CAMERA_EFFECT_SEPIA },
        { NULL, 0 }
    };
  const struct str_map scene_mode_map[] = {
        { "auto", CAMERA_SCENE_MODE_AUTO },
        { "night", CAMERA_SCENE_MODE_NIGHT },
        { NULL, 0 }
   };
  const struct str_map camera_id_map[] = {
        { "back_camera", CAMERA_CAMERA_ID_BACK},
        { "front_camera", CAMERA_CAMERA_ID_FRONT},
        { NULL, 0 }
   };

   const struct str_map zoom_map[] = {
        { "0", CAMERA_ZOOM_1X },
        { "1", CAMERA_ZOOM_2X },
        { "2", CAMERA_ZOOM_3X },
        { "3", CAMERA_ZOOM_4X },
        { NULL, 0 }
   };
   const struct str_map brightness_map[] = {
        { "0", CAMERA_BRIGHTNESS_0 },
        { "1", CAMERA_BRIGHTNESS_1 },
        { "2", CAMERA_BRIGHTNESS_2 },
        { "3", CAMERA_BRIGHTNESS_3 },
        { "4", CAMERA_BRIGHTNESS_4 },
        { "5", CAMERA_BRIGHTNESS_5 },
        { "6", CAMERA_BRIGHTNESS_6 },        
        { NULL, 0 }
   };   
   const struct str_map contrast_map[] = {
        { "0", CAMERA_CONTRAST_0 },
        { "1", CAMERA_CONTRAST_1 },
        { "2", CAMERA_CONTRAST_2 },
        { "3", CAMERA_CONTRAST_3 },
        { "4", CAMERA_CONTRAST_4 },
        { "5", CAMERA_CONTRAST_5 },
        { "6", CAMERA_CONTRAST_6 },        
        { NULL, 0 }
   };     

struct config_element{
	const char *const key;
	const char *const value;
};
struct config_element sprd_front_camera_hardware_config[] = {
        {"whitebalance-values",
         "auto,incandescent,fluorescent,daylight,cloudy-daylight"},
        {"whitebalance", "auto"},
        {"picture-size-values",
         "640x480"},
         {"picture-size", "640x480"},
	{"preview-size-values", 
	 "480x320,352x288,176x144"},
	 {"preview-size", "480x320"},  
	{"preview-format-values", "yuv420sp"},
	{"preview-format", "yuv420sp"},
	{"picture-format-values", "jpeg"},
	{"picture-format", "jpeg"},
	{"preview-frame-rate-values", "15"},
	{"preview-frame-rate", "15"},
	{"jpeg-thumbnail-size-values", "176x144,0x0"},
	{"jpeg-thumbnail-width","176"},
	{"jpeg-thumbnail-height", "144"},
        {"effect-values",
         "none,mono,negative,sepia,antique"},
         {"effect", "none"},
        {"scene-mode-values",
         "auto,night"},
        {"scene-mode", "auto"},
        {"cameraid-values",
         "back_camera,front_camera"},
        {"cameraid",
         "front_camera"},
        {"zoom-supported", "true"},
        {"smooth-zoom-supported", "false"},
        {"max-zoom", "3"},
        {"zoom-ratios", "100,200,300,400"},
        {"zoom", "0"},
        {"brightness-supported", "true"},
        {"smooth-brightness-supported", "false"},
        {"max-brightness", "6"},
        {"brightness-values", "0,1,2,3,4,5,6"},
        {"brightness", "3"},	
        {"contrast-supported", "true"},
        {"smooth-contrast-supported", "false"},
        {"max-contrast", "6"},
        {"contrast-values", "0,1,2,3,4,5,6"},
        {"contrast", "3"}, 
	{"focus-mode-values", "auto"},
	{"focus-mode", "auto"},
	{"min-exposure-compensation", "0"},
	{"max-exposure-compensation", "0"},
	{"exposure-compensation","0"},
	{"exposure-compensation-step", "0"},
	{"sensororientation", "0"},
	{"sensorrotation", "0"},
	{"focal-length", "2.79"}, //???
	{"horizontal-view-angle", "54"}, //???
	{"vertical-view-angle", "54"} //???
};

struct config_element sprd_back_camera_hardware_config[] = {
	//added for NEWMS00120899
	{"flash-mode-values", "off,on"},
	{"flash-mode", "off"},
	{"flash-mode-supported", "true"},
	//
	{"whitebalance-values",
         "auto,incandescent,fluorescent,daylight,cloudy-daylight"},
	{"whitebalance", "auto"},         
#ifdef CONFIG_CAMERA_2M
	{"picture-size-values", 
	 "1600x1200,1280x960,640x480"},
	 {"picture-size", "1600x1200"},  
#else //for 3M
	{"picture-size-values", 
	 "2048x1536,1600x1200,1280x960,640x480"},
	 {"picture-size", "2048x1536"},  
#endif
	{"preview-size-values", 
	 "480x320,352x288,176x144"},
	 {"preview-size", "480x320"}, 
	{"preview-format-values", "yuv420sp"},
	{"preview-format", "yuv420sp"},
	{"picture-format-values", "jpeg"},
	{"picture-format", "jpeg"},
	{"preview-frame-rate-values", "15"},
	{"preview-frame-rate", "15"},
	{"jpeg-thumbnail-size-values", "176x144,0x0"},
	{"jpeg-thumbnail-width","176"},
	{"jpeg-thumbnail-height", "144"},
	{"effect-values",
         "none,mono,negative,sepia,antique"},
         {"effect", "none"},   
	{"scene-mode-values",
	 "auto,night"},
	{"scene-mode", "auto"},
	{"cameraid-values",
	 "back_camera,front_camera"},	
        {"cameraid",
	 "back_camera"},	 
	{"zoom-supported", "true"},	
	{"smooth-zoom-supported", "false"},	
	{"max-zoom", "3"},	
	{"zoom-ratios", "100,200,300,400"},	
	{"zoom", "0"},
        {"brightness-supported", "true"},
        {"smooth-brightness-supported", "false"},
        {"max-brightness", "6"},
        {"brightness-values", "0,1,2,3,4,5,6"},
        {"brightness", "3"},	
        {"contrast-supported", "true"},
        {"smooth-contrast-supported", "false"},
        {"max-contrast", "6"},
        {"contrast-values", "0,1,2,3,4,5,6"},
        {"contrast", "3"}, 
	{"focus-mode-values", "auto"},
	{"focus-mode", "auto"},
	{"min-exposure-compensation", "0"},
	{"max-exposure-compensation", "0"},
	{"exposure-compensation","0"},
	{"exposure-compensation-step", "0"},
	{"focal-length", "3.79"},
	{"sensororientation", "0"},
	{"sensorrotation", "0"},
	{"horizontal-view-angle", "54"}, //???
	{"vertical-view-angle", "54"} //???
};	

#endif //_SPRD_CAMERA_HARDWARE_CONFIG_H_
