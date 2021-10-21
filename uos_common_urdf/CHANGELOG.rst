^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uos_common_urdf
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add missing xacro tag to property
  find . -iname "*.xacro" | xargs sed -i 's#<\([/]\?\)\(if\|unless\|include\|arg\|property\|macro\|insert_block\)#<\1xacro:\2#g'
* Add missing xacro tags for inertials with origin (`#25 <https://github.com/uos/uos_tools/issues/25>`_)

1.0.0 (2020-04-25)
------------------
* Initial version 1.0.0
* Contributors: André Potenza, Jochen Sprickerhof, Martin Günther, Sebastian Pütz, Michael Görner, Tristan Igelbrink
