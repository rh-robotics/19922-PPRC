echo "Pulling submodules..."
echo "---------------------"
# Initialize the submodules and then update
git submodule init
git submodule update --recursive --remote

echo "Finished!"
