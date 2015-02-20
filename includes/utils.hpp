#include <iostream>
#include <sstream>
#include <string>

#ifndef __UTILS_HPP__
#define __UTILS_HPP__

char _getchar() {
	char k;
	std::cin >> k;
	return k;
}

bool response_affirmative() {
	char k = _getchar();
	if (k == 'y' || k == 'Y')
		return true;
	return false;
}

std::string make_log_file_name(const std::string log_file_root, const std::string log_file_type, const std::string log_file_ext) {
	std::ostringstream log_file_name;
	log_file_name << log_file_root << '_' << log_file_type << '.' << log_file_ext;
	return log_file_name.str();
}

#endif //__UTILS_HPP__