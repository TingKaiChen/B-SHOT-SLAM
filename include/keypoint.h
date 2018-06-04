#ifndef KEYPOINT_H
#define KEYPOINT_H

#include "common_include.h"
#include "bshot_bits.h"

namespace myslam{
	class Keypoint{
		public:
			typedef shared_ptr<Keypoint> Ptr;
			Keypoint();
			Keypoint(
				unsigned long id, 
				Vector3f& position,
				bshot_descriptor& descriptor);

			inline Vector3f getPosition() const{return pos_;};
			inline bshot_descriptor getDescriptor() const{return descriptor_;};
			inline unsigned long getId() const{return id_;};

			// static Keypoint::Ptr createKeypoint();
			static Keypoint::Ptr createKeypoint(Vector3f& pos, bshot_descriptor descriptor);

		private:
			unsigned long id_;
			static unsigned long factory_id_;
			Vector3f pos_;
			bshot_descriptor descriptor_;
	};
}

#endif