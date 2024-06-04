import rospy

def main():
    print("A Running")
    rospy.init_node('example_node')
    rospy.spin()

if __name__ == "__main__":
    main()
