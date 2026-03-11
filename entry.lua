local self_ID = "tetet_ch46d_flyable"
declare_plugin(self_ID,
{
dirName			= current_mod_path,
displayName		= _("CH-46D Flyable"),
fileMenuName	= _("CH-46D Flyable"),
state			= "installed",
--developerName	= _(" "),
info			= _("The CH-46D Flyable is a utility helicopter used by the US Army."),
encyclopedia_path = current_mod_path..'/Encyclopedia',

binaries =
{
'CH46D',
},
Skins =
{
	{
		name	= "CH-46D Flyable",
		dir		= "Skins/1"
	},
},
Missions =
{
	{
		name	= _("CH-46D Flyable"),
		dir		= "Missions",
	},
},
LogBook =
{
	{
		name	= _("CH-46D Flyable"),
		type	= "CH-46D Flyable",
	},
},
InputProfiles =
{
	["vwv_ch46d_flyable"] = current_mod_path .. '/Input',
},
Options =
{
    {
        name		= _("CH-46D Flyable"),
        nameId		= "CH-46D Flyable Options",
        dir			= "Options",
        CLSID		= "{CH-46D Flyable options}",
		--allow_in_simulation = true
    },
},
})
-------------------------------------------------------------------------------
-- mount_vfs_model_path(current_mod_path.."/Shapes")
mount_vfs_model_path(current_mod_path.."/Cockpit/Shapes")
--mount_vfs_liveries_path (current_mod_path.."/Liveries")
-- mount_vfs_texture_path(current_mod_path.."/Textures")
-- mount_vfs_texture_path(current_mod_path.."/Textures/AH6J_ExternalTextures")
mount_vfs_texture_path(current_mod_path.."/Cockpit/Textures/AH6J_CockpitTextures")
mount_vfs_texture_path(current_mod_path.."/Cockpit/Textures/Avionics")--for textures used in cockpit systems i.e. digital fonts
mount_vfs_texture_path(current_mod_path.."/Skins/1/ME")

add_aircraft(dofile(current_mod_path..'/Database/vwv_ch46d_early.lua'))
add_aircraft(dofile(current_mod_path..'/Database/vwv_ch46d_late.lua'))
-- dofile(current_mod_path..'/AH-6J.lua')
-- dofile(current_mod_path..'/Weapons/AH6_Weapons.lua')
-- dofile(current_mod_path..'/UnitPayloads/AH-6.lua')
dofile(current_mod_path.."/Views.lua")
make_view_settings('vwv_ch46d_flyable', ViewSettings, SnapViews)

local cfg_path = current_mod_path.."/FM_Config.lua"
dofile(cfg_path)
EFM[1] 				= self_ID
EFM[2] 				= 'CH46D'
EFM.config_path 	= cfg_path

make_flyable('vwv_ch46d_flyable', current_mod_path..'/Cockpit/Scripts/', EFM, current_mod_path..'/comm.lua')

plugin_done()
