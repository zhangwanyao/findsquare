#ifndef CONFIG_PARSE_HPP
#define CONFIG_PARSE_HPP
#include <fstream>
#include <string>
#include <iostream>
#include <iomanip>
#include <map>
#include "log.h"
#include "common_struct.h"
/**
* \brief  class for config information parsing
*/
class ConfigParse
{
public:

	/**
	* \brief constructor of class
	*/
	ConfigParse()
	{
		this->freeMemory();
		this->init();
	}

	/**
	* \brief destructor
	*/
	~ConfigParse()
	{
		this->freeMemory();
	}

	/**
	* \brief trim the whitespace characters before and after
	*/
	inline void WhitespaceTrim(std::string& str)
	{
		if (str.empty())
		{
			return;
		}
		int i, begin, end;
		for (i = 0; i < str.size(); ++i)
		{
			bool is_space = (' ' == str[i]) || ('\t' == str[i]);
			if (!is_space)
			{
				break;
			}
		}
		if (i == str.size())
		{
			str = "";
			return;
		}
		begin = i;
		for (i = (int)str.size() - 1; i >= 0; --i)
		{
			bool is_space = (' ' == str[i]) || ('\t' == str[i]);
			if (!is_space)  break;
		}
		end = i;
		str = str.substr(begin, end - begin + 1);
	}

	/**
	* \brief get configure section and string map from config.ini
	*
	*  @return if success or fail
	*/
	inline bool GetConfigMap(const std::string& filename)
	{
		std::ifstream input_file(filename.c_str());
		if (!input_file)
		{
			log_error("can not open file %s", filename.c_str());
			return false;
		}
		std::string line, config_key, config_value, section;
		std::map<std::string, std::string> config_kv; // current key and value per seciton
		std::map<std::string, std::map<std::string, std::string> >::iterator config_map_it; // current iterator of section
		std::vector<std::string> key_name_list;

		int section_idx = 0;
		int config_idx = 0;
		key_name_list.clear();
		key_name_list.shrink_to_fit();
		while (std::getline(input_file, line))
		{
			/*std::cout << line << std::endl;*/

			if (ParseLine(line, section, config_key, config_value))
			{
				config_map_it = configure_.find(section);
				if (config_map_it == configure_.end())
				{
					// insert the current configure section pair  while finding section
					config_kv.clear();
					configure_.insert(std::make_pair(section, config_kv));
					config_seciton_.push_back(section);
					if (!key_name_list.empty())
					{
						config_key_name_.push_back(key_name_list);
						key_name_list.clear();
						key_name_list.shrink_to_fit();
					}
					config_idx = 0;
					section_idx++;
					//log_info("section_idx =%d section = [%s] ", section_idx, section.c_str());
				}
				else
				{
					// add the key and value per section while finding key and value
					//config_kv.insert(std::make_pair(config_key, config_value));
					config_kv[config_key] = config_value;
					config_map_it->second = config_kv;
					key_name_list.push_back(config_key);
					//log_info("section_idx =%d  config_idx =%d config_key = [%s] ", section_idx, config_idx, config_key.c_str());
					config_idx++;
				}
			}
			config_key.clear();
			config_value.clear();
		}

		if (!key_name_list.empty())
		{
			config_key_name_.push_back(key_name_list);
			key_name_list.clear();
			key_name_list.shrink_to_fit();
		}
		input_file.close();
		return true;
	}

	/**
	* \brief Retrieve the configuration information as a string
	*
	*  @return if success or fail
	*/
	inline bool ReadConfigstr(const char* section, const char* key_item, std::string& str_value)
	{
		std::string tmp_s(section);
		std::string tmp_i(key_item);
		std::map<std::string, std::string> config_kv;
		std::map<std::string, std::string>::iterator config_kv_it;
		std::map<std::string, std::map<std::string, std::string> >::iterator config_it;
		config_it = configure_.find(tmp_s);
		if (config_it == configure_.end())
		{
			return false;
		}
		config_kv = config_it->second;
		config_kv_it = config_kv.find(tmp_i);
		if (config_kv_it == config_kv.end())
		{
			return false;
		}
		str_value.assign(config_kv_it->second.c_str());
		return true;
	}
	/**
	* \brief Retrieve the configuration information in integer form
	*  @return if success or fail
	*/
	inline bool ReadConfigInt(const char* section, const char* key_item, int& int_value)
	{
		std::string tmp_s(section);
		std::string tmp_i(key_item);
		std::map<std::string, std::string> config_kv;
		std::map<std::string, std::string>::iterator config_kv_it;
		std::map<std::string, std::map<std::string, std::string> >::iterator config_it;
		config_it = configure_.find(tmp_s);
		if (config_it == configure_.end())
		{
			return false;
		}
		config_kv = config_it->second;
		config_kv_it = config_kv.find(tmp_i);
		if (config_kv_it == config_kv.end())
		{
			return false;
		}
		int_value = strtol(config_kv_it->second.c_str(), NULL, 0);
		return true;
	}
	/**
	* \brief Retrieve the configuration information in float form
	*  @return if success or fail
	*/
	inline bool ReadConfigFloat(const char* section, const char* key_item, float& float_value)
	{
		std::string tmp_s(section);
		std::string tmp_f(key_item);
		std::map<std::string, std::string> config_kv;
		std::map<std::string, std::string>::iterator config_kv_it;
		std::map<std::string, std::map<std::string, std::string> >::iterator config_it;
		config_it = configure_.find(tmp_s);
		if (config_it == configure_.end())
		{
			return false;
		}
		config_kv = config_it->second;
		config_kv_it = config_kv.find(tmp_f);
		if (config_kv_it == config_kv.end())
		{
			return false;
		}
		float_value = strtof(config_kv_it->second.c_str(), NULL);
		return true;
	}

	/**
	* \brief parse string  of the sections , config keys and config values per line of config.ini
	*
	*  @return if success or fail
	*/
	inline bool ParseLine(const std::string line, std::string& section, std::string& config_key, std::string& config_value)
	{
		if (line.empty()) return false;
		int begin = 0, end = (int)line.size() - 1, cur_pos, section_begin, section_end;

		// check if content of line is comment
		cur_pos = (int)line.find("#");
		if (cur_pos != -1)
		{
			if (begin == cur_pos)
			{
				// return false wile begin char is "#"
				return false;
			}
			end = cur_pos - 1;
		}

		// check if the current line is section
		section_begin = (int)line.find("[");
		section_end = (int)line.find("]");
		if ((section_begin != -1) && (section_end != -1))
		{
			section = line.substr(section_begin + 1, section_end - 1);
			return true;
		}

		// check if the current line have configure key and value
		std::string new_line = line.substr(begin, end + 1 - begin);
		cur_pos = (int)line.find("=");
		if (cur_pos == -1)
		{
			return false;
		}

		config_key = new_line.substr(0, cur_pos);
		WhitespaceTrim(config_key);
		if (config_key.empty())
		{
			return false;
		}
		config_value = new_line.substr(cur_pos + 1, end + 1 - (cur_pos + 1));
		WhitespaceTrim(config_value);
		cur_pos = (int)config_value.find("\r");
		if (cur_pos > 0) config_value.replace(cur_pos, 1, "");
		cur_pos = (int)config_value.find("\n");
		if (cur_pos > 0) config_value.replace(cur_pos, 1, "");
		return true;
	}

	/**
	* \brief parse  system configure value from SYS_CNTRL_PARA section of file config.ini
	*
	*  @return if success or fail
	*/
	inline bool ParseConfigure(SysCntrlParams& sys_control_para)
	{
		int tmp_i = std::numeric_limits<int>::lowest();
		bool sync_con = ReadConfigInt("SYS_CNTRL_PARA", "is_cad", tmp_i);
		if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) sys_control_para.is_cad = (tmp_i == 0) ? false : true;
		sync_con = ReadConfigInt("SYS_CNTRL_PARA", "scanner_type", tmp_i);
		if ((sync_con) && (tmp_i < SCANNER_TYPE_MAX) && (tmp_i > INVALID_SCAN_TYPE)) sys_control_para.scanner_type = (ScannerType)tmp_i;
		sync_con = ReadConfigInt("SYS_CNTRL_PARA", "encryption_type", tmp_i);
		if ((sync_con) && (tmp_i < 2) && (tmp_i >= 0)) sys_control_para.encryption_type = tmp_i;
		return true;
	}
	/**
	* \brief parse  voxel configure value from VOXEL_SIZE_CONFIG section of file config.ini
	*  @return if success or fail
	*/
	inline bool ParseConfigure(VoxelParams& voxel_para)
	{
		float tmp_f = std::numeric_limits<float>::infinity();
		bool sync_con = ReadConfigFloat("VOXEL_SIZE_CONFIG", "voxel_x_length", tmp_f);
		if ((sync_con) && (tmp_f >= 0)) voxel_para.length_x_of_voxel = tmp_f;
		sync_con = ReadConfigFloat("VOXEL_SIZE_CONFIG", "voxel_y_length", tmp_f);
		if ((sync_con) && (tmp_f >= 0)) voxel_para.length_y_of_voxel = tmp_f;
		sync_con = ReadConfigFloat("VOXEL_SIZE_CONFIG", "voxel_z_length", tmp_f);
		if ((sync_con) && (tmp_f >= 0)) voxel_para.length_z_of_voxel = tmp_f;
		return true;
	}

	/**
	* \brief parse  system debug  configure parameters from SYS_DBG_PARA section of file config.ini
	*  @return if success or fail
	*/
	inline bool ParseConfigure(SysDebugParams& sys_dbg_para)
	{
		int tmp_i = std::numeric_limits<int>::lowest();
		float tmp_f = std::numeric_limits<float>::infinity();
		bool sync_con = ReadConfigInt("SYS_DBG_PARA", "reserved_int0", tmp_i);
		if (sync_con)  sys_dbg_para.reserved_int0 = tmp_i;
		sync_con = ReadConfigInt("SYS_DBG_PARA", "reserved_int1", tmp_i);
		if (sync_con) sys_dbg_para.reserved_int1 = tmp_i;
		sync_con = ReadConfigFloat("SYS_DBG_PARA", "reserved_float0", tmp_f);
		if (sync_con)  sys_dbg_para.reserved_float0 = tmp_f;
		sync_con = ReadConfigFloat("SYS_DBG_PARA", "reserved_float1", tmp_f);
		if (sync_con) sys_dbg_para.reserved_float1 = tmp_f;
		return true;
	}

	/**
	* \brief save info from file config.ini (file configinfo.txt)
	*  @return if success or fail
	*/
	inline bool SaveConfigInfo(const std::string& config_file)
	{
		std::ofstream config_outinfo(config_file);
		config_outinfo << std::setprecision(6) << std::fixed;
		if (config_key_name_.size() != config_seciton_.size())
		{
			log_error("config section number %d is not equal to config key list size %d", config_seciton_.size(), config_key_name_.size());
			return false;
		}

		for (int i = 0; i < config_key_name_.size(); i++)
		{
			config_outinfo << "[" << config_seciton_[i] << "]" << std::endl;
			for (int j = 0; j < config_key_name_[i].size(); j++)
			{
				std::string key_name;

				if (ReadConfigstr(config_seciton_[i].c_str(), config_key_name_[i][j].c_str(), key_name))
				{
					config_outinfo << config_key_name_[i][j] << "=" << key_name << std::endl;
					//log_debug("section[%d] %s config key[%d] %s = %s", i, config_seciton_[i].c_str(), j, config_key_name_[i][j].c_str(), key_name.c_str());
				}
				else
				{
					//log_error("section[%d] %s config key[%d] = %s don't find ",i, config_seciton_[i],j, config_key_name_[i][j]);
					continue;
				}
			}
			config_outinfo << std::endl;
		}
		return true;
	}

	/**
	* \brief save system control para (file configParseInfo.txt)
	*/
	virtual inline void SaveConfigParams(const std::string& config_file, const SysCntrlParams sys_control_para)
	{
		std::ofstream config_outinfo(config_file);
		//output config of SEG_CNTRL_PARA
		config_outinfo << "[" << "SYS_CNTRL_PARA" << "]" << std::endl;
		config_outinfo << "is_cad" << "=" << sys_control_para.is_cad << std::endl;
		config_outinfo << "scanner_type" << "=" << sys_control_para.scanner_type << std::endl;
		config_outinfo << "encryption_type" << "=" << sys_control_para.encryption_type << std::endl;
	}

	/**
	* \brief save voxel size para (file configParseInfo.txt)
	*/
	virtual inline void SaveConfigParams(const std::string& config_file, const VoxelParams voxel_para)
	{
		std::ofstream config_outinfo(config_file, std::ios::app);
		//output config of section VOXEL_SIZE_CONFIG
		config_outinfo << "[" << "VOXEL_SIZE_CONFIG" << "]" << std::endl;
		config_outinfo << "voxel_x_length" << "=" << voxel_para.length_x_of_voxel << std::endl;
		config_outinfo << "voxel_y_length" << "=" << voxel_para.length_y_of_voxel << std::endl;
		config_outinfo << "voxel_z_length" << "=" << voxel_para.length_z_of_voxel << std::endl;
		config_outinfo << std::endl;
	}

	/**
	* \brief save  system debug  configure parameters (file configParseInfo.txt)
	*/
	virtual inline void SaveConfigParams(const std::string & config_file, const SysDebugParams &sys_dbg_para)
	{
		std::ofstream config_outinfo(config_file, std::ios::app);

		//output config of section VOXEL_SIZE_CONFIG
		config_outinfo << "[" << "SYS_DBG_PARA" << "]" << std::endl;
		config_outinfo << "reserved_int0" << "=" << sys_dbg_para.reserved_int0 << std::endl;
		config_outinfo << "reserved_int1" << "=" << sys_dbg_para.reserved_int1 << std::endl;
		config_outinfo << "reserved_float0" << "=" << sys_dbg_para.reserved_float0 << std::endl;
		config_outinfo << "reserved_float1" << "=" << sys_dbg_para.reserved_float1 << std::endl;
		config_outinfo << std::endl;
	}

protected:
	/**
	* \brief Parse Class member init
	*
	*  @return if success or fail
	*/
	virtual inline bool init()
	{
		return true;
	}
	/**
	* \brief release memory of Parse Class
	*
	*  @return if success or fail
	*/
	virtual inline bool freeMemory()
	{
		if (!configure_.empty())
		{
			configure_.clear();
		}
		if (!config_key_name_.empty())
		{
			for (int i = 0; i < config_key_name_.size(); i++)
			{
				config_key_name_[i].clear();
				config_key_name_[i].shrink_to_fit();
			}
			config_key_name_.clear();
			config_key_name_.shrink_to_fit();
		}
		if (!config_seciton_.empty())
		{
			config_seciton_.clear();
			config_seciton_.shrink_to_fit();
		}
		return true;
	}
	/**
	* \brief config map
	*/
	std::map<std::string, std::map<std::string, std::string> >configure_;
	/**
	* \brief config key and name list
	*/
	std::vector<std::vector<std::string>> config_key_name_;
	/**
	* \brief config seciton list
	*/
	std::vector<std::string> config_seciton_;
};

#endif // CONFIG_PARSE_H