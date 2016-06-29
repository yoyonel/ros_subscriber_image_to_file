// urls:
// - http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages
// - -> a lire: http://wiki.ros.org/image_transport/Tutorials (pas réellement utilisé ici encore)
// - http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages
//   - http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>


class ImageToFile {
protected:
    ros::NodeHandle nh_;

private:
    std::string prefix_;

    bool b_show_image_;

public:
    std::string image_topic_;

    image_transport::Subscriber sub_;

    bool writeImage(
            const sensor_msgs::ImageConstPtr& _msg,
            const cv::Mat& _image
            )
    {
        // On récupère le timestamp du message
        ros::Time stamp = _msg->header.stamp;
        // On construit le nom du fichier image par rapport à ce timestamp
        std::stringstream ss_image_filename;
        ss_image_filename << prefix_ << _msg->header.stamp << ".jpg";
        // On écrit l'image
        cv::imwrite( ss_image_filename.str(), _image );
        // Debug info
        ROS_INFO_STREAM("Write image file: " << ss_image_filename.str());
    }

    void image_cb(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

            writeImage( msg, image );

            if( b_show_image_ )
            {
                cv::imshow("view", image);
                cv::waitKey(30);
            }
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    ImageToFile (
            ros::NodeHandle _priv_nh = ros::NodeHandle("~"),
            bool _b_show_image=false)
    {
        // Check if a prefix parameter is defined for output file names.
        ;

        if (_priv_nh.getParam ("prefix", prefix_))
        {
            ROS_INFO_STREAM ("JPG file prefix is: " << prefix_);
        }
        else if (nh_.getParam ("prefix", prefix_))
        {
            ROS_WARN_STREAM ("Non-private JPG prefix parameter is DEPRECATED: "
                             << prefix_);
        }
        else {
            prefix_ = "IMG_";
        }

        b_show_image_ = _b_show_image;

        image_topic_ = "camera/image";

        image_transport::ImageTransport it(nh_);
        sub_ = it.subscribe(image_topic_, 1, &ImageToFile::image_cb, this);

        ROS_INFO ("Listening for incoming data on topic %s",
                  nh_.resolveName (image_topic_).c_str ());
    }


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");

    ros::NodeHandle priv_nh("~");

    bool b_show_image;
    if ( priv_nh.getParam("show_image", b_show_image) && b_show_image )
    {
        ROS_INFO_STREAM("Show image recorded");
        cv::namedWindow("view");
        cv::startWindowThread();
    }

    ImageToFile itof(priv_nh, b_show_image);

    ros::spin();

    cv::destroyWindow("view");
}
