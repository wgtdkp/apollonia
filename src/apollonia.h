#pragma once

#define DISABLE_COPY_AND_ASSIGN(Type)               \
  Type(const Type& other) = delete;                 \
  const Type& operator=(const Type& other) = delete;

namespace apollonia {

}
