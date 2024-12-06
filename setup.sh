if [ "$CONDA_DEFAULT_ENV" != "drones" ]
then
    echo You must have the drones environment from gym-pybullet-drones sourced first.
else
    export PYTHONPATH=$(dirname -- "$( readlink -f -- "$0"; )";)
    pip install -r $PYTHONPATH/requirements.txt
fi

