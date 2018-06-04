#include "keypoint.h"

namespace myslam{
	unsigned long Keypoint::factory_id_ = 0;

	Keypoint::Keypoint(): id_(-1), pos_(Vector3f(0, 0, 0)){
		// Warning: the descriptor is not initialized
	}

	Keypoint::Keypoint(unsigned long id, Vector3f& position, bshot_descriptor& descriptor)
		: id_(id), pos_(position), descriptor_(descriptor){

	}

	// Keypoint::Ptr createKeypoint(){
	// 	return make_shared<Keypoint>(factory_id_++, Vector3f(0, 0, 0), );
	// }

	Keypoint::Ptr Keypoint::createKeypoint(Vector3f& pos, bshot_descriptor descriptor){
		return make_shared<Keypoint>(factory_id_++, pos, descriptor);
	}
}