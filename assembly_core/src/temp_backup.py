                        self.assembly_task(
                            obj_dict,
                            temp_obj, avail_parent_holepin_names,
                            target_obj, avail_child_holepin_names
                        )


    def assembly_task(self, obj_dict, parent_obj, parent_holepin_names, child_obj, child_holepin_names):
        tr, quat, ref_axis, criterion, can_assembly = self.AT.get_AsmPose_by_HolePin(
            parent_obj, parent_holepin_names, child_obj, child_holepin_names
        )
        if not can_assembly:
            rospy.logwarn("Given assembly task can't be done!: \nparent_holepin_names: {}\nchild_holepin_names: {}".format(\
                parent_holepin_names, child_holepin_names))
            return TypeError
        else:
            sub_name = parent_obj.assem_name
            sub_obj = self.AT.try_attaching(
                parent_obj, parent_holepin_names,
                child_obj, child_holepin_names,
                tr, quat, criterion, sub_name, verbose=1
            )

            self.add_subObject(
                sub_obj,
                parent_obj.assem_name,
                child_obj.assem_name,
                obj_dict
            )
    
    def attach_task(self, obj_dict, parent_obj, parent_holepin_names, child_obj, child_holepin_names):
        tr, quat, ref_axis, criterion, can_attach = self.AT.get_AttachPose(
            parent_obj, parent_holepin_names, child_obj, child_holepin_names
        )
        if not can_attach:
            rospy.logwarn("Given attach task can't be done!: \nparent_holepin_names: {}\nchild_holepin_names: {}".format(\
                parent_holepin_names, child_holepin_names))
            return TypeError
        else:
            sub_name = parent_obj.assem_name
            sub_obj = self.AT.just_attaching(
                parent_obj, parent_holepin_names,
                child_obj, child_holepin_names,
                tr, quat, criterion, sub_name, verbose=1
            )

            self.add_subObject(
                sub_obj,
                parent_obj.assem_name,
                child_obj.assem_name,
                obj_dict
            )