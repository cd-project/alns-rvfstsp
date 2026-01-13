#!/bin/bash
folder_path="/Users/cuong/CLionProjects/ALNS_RV_FSTSP/uniform_extra"
binary_file="/Users/cuong/CLionProjects/ALNS_RV_FSTSP/cmake-build-performance/RV-FSTSP_heuristic_250k_75k"
dtl_values=(00)
#temp=369
#cr=99998
#minR=20
#maxR=40
#dci=79
#dcr=87
#bestscore=58
#imprscore=32
#accscore=25
#rejscore=10
#i1=18
#i2=22
#r1=36
#r2=14
#r3=14
#r4=16
#r5=13
#r6=41
#r7=12
temp=134
cr=99997
minR=16
maxR=40
dci=81
dcr=65
bestscore=46
imprscore=51
accscore=20
rejscore=34
i1=43
i2=32
r1=14
r2=47
r3=79
r4=77
r5=43
r6=22
r7=61
for val in "${dtl_values[@]}"; do

    # Iterate through files in the directory
    find "$folder_path" -maxdepth 1 | while IFS= read -r folder; do
        # Extract the folder name from the full path
        folder_name=$(basename "$folder")
#        output="/home/cuong/CLionProjects/RV-FSTSP-heuristic/logs/${folder_name}.out"
#        touch "$output"
        # Call the binary file with the folder name as an argument
        echo "$folder_path/$folder_name"
        "$binary_file" "-i" "$folder_path/$folder_name" "-temp" "$temp" "-cr" "$cr" "-minR" "$minR" "-maxR" "$maxR" "-dci" "$dci" "-dcr" "$dcr" "-bestscore" "$bestscore" "-impscore" "$imprscore" "-accscore" "$accscore" "-rejscore" "$rejscore" "-i1" "$i1" "-i2" "$i2" "-r1" "$r1" "-r2" "$r2" "-r3" "$r3" "-r4" "$r4" "-r5" "$r5" "-r6" "$r6" "-r7" "$r7"
#        exit_status=$?
#            if [ $exit_status -eq 255 ] || [ $exit_status -eq 134 ] || [ $exit_status -eq 3 ]
#            then
#                echo "C++ program terminated abnormally with status $exit_status. Terminating the script."
#                exit 1  # Exit the shell script with a non-zero status
#            fi
    done
done
