#ifndef ORBVOCABULARY_H
#define ORBVOCABULARY_H

#include "Thirdparty/DBoW2/DBoW2/FORB.h"
#include "Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace semantic_slam
{

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
  ORBVocabulary;

} //namespace semantic_slam

#endif // ORBVOCABULARY_H
