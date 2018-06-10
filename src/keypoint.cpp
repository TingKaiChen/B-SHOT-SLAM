#include "keypoint.h"

namespace myslam{
	unsigned long Keypoint::factory_id_ = 0;

	Keypoint::Keypoint(): id_(-1), pos_(Vector3f(0, 0, 0)){
		// Warning: the descriptor is not initialized
	}

	Keypoint::Keypoint(unsigned long id, Vector3f& position, float& seg_ratio, bshot_descriptor& descriptor)
		: id_(id), pos_(position), seg_ratio_(seg_ratio), descriptor_(descriptor){

	}

	// Keypoint::Ptr createKeypoint(){
	// 	return make_shared<Keypoint>(factory_id_++, Vector3f(0, 0, 0), );
	// }

	// Keypoint::Ptr Keypoint::createKeypoint(Vector3f& pos, bshot_descriptor descriptor){
	// 	return make_shared<Keypoint>(factory_id_++, pos, descriptor);
	// }

	Keypoint::Ptr Keypoint::createKeypoint(Vector3f& pos, float seg_ratio, bshot_descriptor descriptor){
		// Downsampling (grid map)
		int prec = 10;	// Precision (mm)
		Vector3f grid_pos(
			int(trunc(pos[0]/prec))*prec, 
			int(trunc(pos[1]/prec))*prec, 
			int(trunc(pos[2]/prec))*prec);

		return make_shared<Keypoint>(factory_id_++, grid_pos, seg_ratio, descriptor);
	}
}