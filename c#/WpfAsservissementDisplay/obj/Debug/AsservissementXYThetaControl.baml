   M S B A M L   `   `   `  ����S  OWpfAsservissementDisplay, Version=1.0.0.0, Culture=neutral, PublicKeyToken=nullC&clr-namespace:WpfAsservissementDisplayWpfAsservissementDisplay  5      ;    5WpfAsservissementDisplay.AsservissementXYThetaControl   5      \ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5      R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5      Jmc;http://schemas.openxmlformats.org/markup-compatibility/2006     5      @d2http://schemas.microsoft.com/expression/blend/2008     5      8local&clr-namespace:WpfAsservissementDisplay     5      .��5	      �� ���#AAAAAA$��1q�6#   ._�5
   
   �   � RowDefinitions  5      �� 5      $
��0.4* �6    60   �� 5      $
��0.4* �6    60   �� 5      $
��0.4* �6    60   �� 5      $
��0.4* �6    60   �� 5      $
��0.4* �6    60   �� 5      $��1* �6    6.   �� 5      $��1* �6    6.   �� 5      $��1* �6    6.    � ColumnDefinitions 5      �� 5      $��3* �6#   60   �� 5      �� 5      �� 5      �� 5      .+�5      �� ���#AAAAAA$��1q�6+   $��8��6?   �� 5      ���#AAAAAA$��1q�6+   $��1��6?   $��8��6L   �� 5       ���#AAAAAA$��1q�6+   $��2��6?   $��8��6L   �� 5!      ���#AAAAAA$��1q�6+   $��3��6?   $��8��6L   �� 5"      ���#AAAAAA$��1q�6+   $��4��6?   $��8��6L   �� 5#      ���#AAAAAA$��1q�6+   $��5��6?   $��8��6L   �� 5$      ���#AAAAAA$��1q�6+   $��7��6?   $��8��6L   �� 5&      -   5         ��Name$ 
LabelTitre��5&      $��Asservissement en ...��6(   
�������$��	0,0,0.2,0q�6[   $��0q�6n   $��16��6z   $��Bottom=�6�   6�   �� 5'      $��1��6   $��Consigne : ��6!   
�������$��0q�6K   $��0q�6V   6d   �� 5(      $��2��6   $��
Measure : ��6!   
�������$��0q�6J   $��0q�6U   6c   �� 5)      $��3��6   $��Error : ��6!   
�������$��0q�6H   $��0q�6S   6a   �� 5*      $��4��6   $��
Command : ��6!   
�������$��0q�6J   $��0q�6U   6c   �� 5+      $��1��6   $��Linear��6$   
�������$��Center=�6H   $��0q�6e   $��0q�6p   6   �� 5,      $��2��6   $��Angular��6$   
�������$��Center=�6I   $��0q�6f   $��0q�6q   6�   �� 5-      $��3��6   $��MG��6$   
�������$��Center=�6D   $��0q�6a   $��0q�6l   6{   �� 5.      $��4��6   $��MD��6$   
�������$��Center=�6D   $��0q�6b   $��0q�6m   6{   �� 51      -   5        $ LabelConsigneX��51      $��0��6,   $��1��68   $��1��6H   
�������$��0q�6h   $��0q�6s   �� 52      -   5        $ LabelConsigneTheta��52      $��0��60   $��2��6<   $��1��6L   
�������$��0q�6l   $��0q�6w   �� 53      -   5        $ LabelConsigneM1��53      $��0��6-   $��3��69   $��1��6I   
�������$��0q�6i   $��0q�6t   �� 54      -   5        $ LabelConsigneM2��54      $��0��6-   $��4��69   $��1��6I   
�������$��0q�6i   $��0q�6t   �� 55      -   5        $ LabelMeasureX��55      $��0��6+   $��1��67   $��2��6G   
�������$��0q�6g   $��0q�6r   �� 56      -   5        $ LabelMeasureTheta��56      $��0��6/   $��2��6;   $��2��6K   
�������$��0q�6k   $��0q�6v   �� 57      -   5        $ LabelMeasureM1��57      $��0��6,   $��3��68   $��2��6H   
�������$��0q�6h   $��0q�6s   �� 58      -	   5        $ LabelMeasureM2��58      $��0��6,   $��4��68   $��2��6H   
�������$��0q�6h   $��0q�6s   �� 59      -
   5        $ LabelErreurX��59      $��0��6*   $��1��66   $��3��6F   
�������$��0q�6f   $��0q�6q   �� 5:      -   5        $ LabelErreurTheta��5:      $��0��6.   $��2��6:   $��3��6J   
�������$��0q�6j   $��0q�6u   �� 5;      -   5        $ LabelErreurM1��5;      $��0��6+   $��3��67   $��3��6G   
�������$��0q�6g   $��0q�6r   �� 5<      -   5        $ LabelErreurM2��5<      $��0��6+   $��4��67   $��3��6G   
�������$��0q�6g   $��0q�6r   �� 5=      -   5        $ LabelCommandX��5=      $��0��6+   $��1��67   $��4��6G   
�������$��0q�6g   $��0q�6r   �� 5>      -   5        $ LabelCommandTheta��5>      $��0��6/   $��2��6;   $��4��6K   
�������$��0q�6k   $��0q�6v   �� 5?      -   5        $ LabelCommandM1��5?      $��0��6,   $��3��68   $��4��6H   
�������$��0q�6h   $��0q�6s   �� 5@      -   5        $ LabelCommandM2��5@      $��0��6,   $��4��68   $��4��6H   
�������$��0q�6h   $��0q�6s   � 5C      $��0��6   $��5��6#     5D      �� 5E      $��1* �6$   �� 5F      $��1* �6$   �� 5G      $��1* �6$   .+�5I      �� $��0��6   $��KP��6%   
�������$��0q�6E   $��0q�6P   �� 5J      $��1��6   $��Correction P��6%   
�������$��0q�6O   $��0q�6Z   �� 5K      $��2��6   $��
Corr P Max��6%   
�������$��0q�6M   $��0q�6X   5L      � 5N      $��0��6   $��6��6#     5O      �� 5P      $��1* �6$   �� 5Q      $��1* �6$   �� 5R      $��1* �6$   .+�5T      �� $��0��6   $��KI��6%   
�������$��0q�6E   $��0q�6P   �� 5U      $��1��6   $��Correction I��6%   
�������$��0q�6O   $��0q�6Z   �� 5V      $��2��6   $��
Corr I Max��6%   
�������$��0q�6M   $��0q�6X   5W      � 5Y      $��0��6   $��7��6#     5Z      �� 5[      $��1* �6$   �� 5\      $��1* �6$   �� 5]      $��1* �6$   .+�5_      �� $��0��6   $��KD��6%   
�������$��0q�6E   $��0q�6P   �� 5`      $��1��6   $��Correction D��6%   
�������$��0q�6O   $��0q�6Z   �� 5a      $��2��6   $��
Corr D Max��6%   
�������$��0q�6M   $��0q�6X   5b      � 5d      -   5         �Name$ 
CorrPXGrid��5d      $��1��6'   $��50��67   $��5��6E    5e      �� 5f      $��1* �6'     5h      �� 5i      $��1* �6$   �� 5j      $��1* �6$   �� 5k      $��1* �6$   .+�5m      �� -   5        $ LabelKpX��5m      $��0��6*   
�������$��0q�6M   $��0q�6X   �� 5n      -   5        $ LabelCorrPX��5n      $��1��6-   $��0��6:   
�������$��0q�6Y   $��0q�6d   �� 5o      -   5        $ LabelCorrMaxPX��5o      $��2��60   $��0��6=   
�������$��0q�6`   $��0q�6k   5p      � 5q      -   5        $ CorrPThetaGrid��5q      $��2��6+   $��50��6;   $��5��6I    5r      �� 5s      $��1* �6'     5u      �� 5v      $��1* �6$   �� 5w      $��1* �6$   �� 5x      $��1* �6$   .+�5z      �� -   5        $ LabelKpTheta��5z      $��0��6.   
�������$��0q�6Q   $��0q�6\   �� 5{      -   5        $ LabelCorrPTheta��5{      $��1��61   $��0��6>   
�������$��0q�6]   $��0q�6h   �� 5|      -   5        $ LabelCorrMaxPTheta��5|      $��2��64   $��0��6A   
�������$��0q�6d   $��0q�6o   5}      � 5      -   5        $ CorrPM1Grid��5      $��3��6(   $��50��68   $��5��6F    5�      �� 5�      $��1* �6'     5�      �� 5�      $��1* �6$   �� 5�      $��1* �6$   �� 5�      $��1* �6$   .+�5�      �� -   5        $ 	LabelKpM1��5�      $��0��6+   
�������$��0q�6N   $��0q�6Y   �� 5�      -   5        $ LabelCorrPM1��5�      $��1��6.   $��0��6;   
�������$��0q�6Z   $��0q�6e   �� 5�      -   5        $ LabelCorrMaxPM1��5�      $��2��61   $��0��6>   
�������$��0q�6a   $��0q�6l   5�      � 5�      -   5        $ CorrPM2Grid��5�      $��4��6(   $��50��68   $��5��6F    5�      �� 5�      $��1* �6'     5�      �� 5�      $��1* �6$   �� 5�      $��1* �6$   �� 5�      $��1* �6$   .+�5�      �� -   5        $ 	LabelKpM2��5�      $��0��6+   
�������$��0q�6N   $��0q�6Y   �� 5�      -    5        $ LabelCorrPM2��5�      $��1��6.   $��0��6;   
�������$��0q�6Z   $��0q�6e   �� 5�      -!   5        $ LabelCorrMaxPM2��5�      $��2��61   $��0��6>   
�������$��0q�6a   $��0q�6l   5�      � 5�      -"   5        $ 
CorrIXGrid��5�      $��1��6'   $��50��67   $��6��6E    5�      �� 5�      $��1* �6'   �� 5�      $��30 �6'     5�      �� 5�      $��1* �6$   �� 5�      $��1* �6$   �� 5�      $��1* �6$   .+�5�      �� -#   5        $ LabelKiX��5�      $��0��6*   
�������$��0q�6M   $��0q�6X   �� 5�      -$   5        $ LabelCorrIX��5�      $��1��6-   $��0��6:   
�������$��0q�6Y   $��0q�6d   �� 5�      -%   5        $ LabelCorrMaxIX��5�      $��2��60   $��0��6=   
�������$��0q�6`   $��0q�6k   5�      � 5�      -&   5        $ CorrIThetaGrid��5�      $��2��6+   $��50��6;   $��6��6I    5�      �� 5�      $��1* �6'     5�      �� 5�      $��1* �6$   �� 5�      $��1* �6$   �� 5�      $��1* �6$   .+�5�      �� -'   5        $ LabelKiTheta��5�      $��0��6.   
�������$��0q�6Q   $��0q�6\   �� 5�      -(   5        $ LabelCorrITheta��5�      $��1��61   $��0��6>   
�������$��0q�6]   $��0q�6h   �� 5�      -)   5        $ LabelCorrMaxITheta��5�      $��2��64   $��0��6A   
�������$��0q�6d   $��0q�6o   5�      � 5�      -*   5        $ CorrIM1Grid��5�      $��3��6(   $��50��68   $��6��6F    5�      �� 5�      $��1* �6'     5�      �� 5�      $��1* �6$   �� 5�      $��1* �6$   �� 5�      $��1* �6$   .+�5�      �� -+   5        $ 	LabelKiM1��5�      $��0��6+   
�������$��0q�6N   $��0q�6Y   �� 5�      -,   5        $ LabelCorrIM1��5�      $��1��6.   $��0��6;   
�������$��0q�6Z   $��0q�6e   �� 5�      --   5        $ LabelCorrMaxIM1��5�      $��2��61   $��0��6>   
�������$��0q�6a   $��0q�6l   5�      � 5�      -.   5        $ CorrIM2Grid��5�      $��4��6(   $��50��68   $��6��6F    5�      �� 5�      $��1* �6'     5�      �� 5�      $��1* �6$   �� 5�      $��1* �6$   �� 5�      $��1* �6$   .+�5�      �� -/   5        $ 	LabelKiM2��5�      $��0��6+   
�������$��0q�6N   $��0q�6Y   �� 5�      -0   5        $ LabelCorrIM2��5�      $��1��6.   $��0��6;   
�������$��0q�6Z   $��0q�6e   �� 5�      -1   5        $ LabelCorrMaxIM2��5�      $��2��61   $��0��6>   
�������$��0q�6a   $��0q�6l   5�      � 5�      -2   5        $ 
CorrDXGrid��5�      $��1��6'   $��50��67   $��7��6E    5�      �� 5�      $��1* �6'     5�      �� 5�      $��1* �6$   �� 5�      $��1* �6$   �� 5�      $��1* �6$   .+�5�      �� -3   5        $ LabelKdX��5�      $��0��6*   
�������$��0q�6M   $��0q�6X   �� 5�      -4   5        $ LabelCorrDX��5�      $��1��6-   $��0��6:   
�������$��0q�6Y   $��0q�6d   �� 5�      -5   5        $ LabelCorrMaxDX��5�      $��2��60   $��0��6=   
�������$��0q�6`   $��0q�6k   5�      � 5�      -6   5        $ CorrDThetaGrid��5�      $��2��6+   $��50��6;   $��7��6I    5�      �� 5�      $��1* �6'     5�      �� 5�      $��1* �6$   �� 5�      $��1* �6$   �� 5�      $��1* �6$   .+�5�      �� -7   5        $ LabelKdTheta��5�      $��0��6.   
�������$��0q�6Q   $��0q�6\   �� 5�      -8   5        $ LabelCorrDTheta��5�      $��1��61   $��0��6>   
�������$��0q�6]   $��0q�6h   �� 5�      -9   5        $ LabelCorrMaxDTheta��5�      $��2��64   $��0��6A   
�������$��0q�6d   $��0q�6o   5�      � 5�      -:   5        $ CorrDThetaM1��5�      $��3��6)   $��50��69   $��7��6G    5�      �� 5�      $��1* �6'     5�      �� 5�      $��1* �6$   �� 5�      $��1* �6$   �� 5�      $��1* �6$   .+�5�      �� -;   5        $ 	LabelKdM1��5�      $��0��6+   
�������$��0q�6N   $��0q�6Y   �� 5�      -<   5        $ LabelCorrDM1��5�      $��1��6.   $��0��6;   
�������$��0q�6Z   $��0q�6e   �� 5�      -=   5        $ LabelCorrMaxDM1��5�      $��2��61   $��0��6>   
�������$��0q�6a   $��0q�6l   5�      � 5�      ->   5        $ CorrDM2Grid��5�      $��4��6(   $��50��68   $��7��6F    5�      �� 5�      $��1* �6'     5�      �� 5�      $��1* �6$   �� 5�      $��1* �6$   �� 5      $��1* �6$   .+�5     �� -?   5        $ 	LabelKdM2��5     $��0��6+   
�������$��0q�6N   $��0q�6Y   �� 5     -@   5        $ LabelCorrDM2��5     $��1��6.   $��0��6;   
�������$��0q�6Z   $��0q�6e   �� 5     -A   5        $ LabelCorrMaxDM2��5     $��2��61   $��0��6>   
�������$��0q�6a   $��0q�6l   5     5     5     5	     