/*
 * fromROSMsg.h
 *
 *  Created on: Nov 28, 2013
 *      Author: robo
 */

#ifndef FROMROSMSG_H_
#define FROMROSMSG_H_

#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/for_each_type.h>
#include <pcl/exceptions.h>
#include <pcl/console/print.h>
#include <boost/foreach.hpp>

using namespace pcl;

template <typename PointT> void
fromROSMsg (const sensor_msgs::PointCloud2& msg, pcl::PointCloud<PointT>& cloud,
		const MsgFieldMap& field_map)
{
	// Copy info fields
	cloud.header.seq   = msg.header.seq;
//	cloud.header.stamp   = msg.header.stamp;
	cloud.header.frame_id   = msg.header.frame_id;
	cloud.width    = msg.width;
	cloud.height   = msg.height;
	cloud.is_dense = msg.is_dense == 1;

	// Copy point data
	uint32_t num_points = msg.width * msg.height;
	cloud.points.resize (num_points);
	uint8_t* cloud_data = reinterpret_cast<uint8_t*>(&cloud.points[0]);

	// Check if we can copy adjacent points in a single memcpy
	if (field_map.size() == 1 &&
			field_map[0].serialized_offset == 0 &&
			field_map[0].struct_offset == 0 &&
			msg.point_step == sizeof(PointT))
	{
		uint32_t cloud_row_step = sizeof(PointT) * cloud.width;
		const uint8_t* msg_data = &msg.data[0];
		// Should usually be able to copy all rows at once
		if (msg.row_step == cloud_row_step)
		{
			memcpy (cloud_data, msg_data, msg.data.size ());
		}
		else
		{
			for (uint32_t i = 0; i < msg.height; ++i, cloud_data += cloud_row_step, msg_data += msg.row_step)
				memcpy (cloud_data, msg_data, cloud_row_step);
		}

	}
	else
	{
		// If not, memcpy each group of contiguous fields separately
		for (uint32_t row = 0; row < msg.height; ++row)
		{
			const uint8_t* row_data = &msg.data[row * msg.row_step];
			for (uint32_t col = 0; col < msg.width; ++col)
			{
				const uint8_t* msg_data = row_data + col * msg.point_step;
				BOOST_FOREACH (const detail::FieldMapping& mapping, field_map)
				{
					memcpy (cloud_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
				}
				cloud_data += sizeof (PointT);
			}
		}
	}
}

template<typename PointT> void
fromROSMsg (const sensor_msgs::PointCloud2& msg, pcl::PointCloud<PointT>& cloud)
{
	MsgFieldMap field_map;
	std::vector<PCLPointField> fields;
	for(int i=0;i<msg.fields.size();++i) {
		PCLPointField pf;
		pf.count=msg.fields[i].count;
		pf.datatype=msg.fields[i].datatype;
		pf.name=msg.fields[i].name;
		pf.offset=msg.fields[i].offset;
		fields.push_back(pf);
	}
	createMapping<PointT> (fields, field_map);
	fromROSMsg (msg, cloud, field_map);
}


#endif /* FROMROSMSG_H_ */
