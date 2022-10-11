#pragma once

#include <sdf/sdf.hh>

template <typename T>
bool AssignSdfParam(const sdf::ElementPtr _sdf, std::string _name, T &_var) {
  bool success;
  std::tie(_var, success) = _sdf->Get<T>(_name, _var);
  return success;
}
