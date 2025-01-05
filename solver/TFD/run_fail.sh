DOMAIN_FILE=$1
PROBLEM_FILE=$2

# Corrected SCRIPT_DIR evaluation without unnecessary quoting
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

rm -rf "$SCRIPT_DIR/output"
mkdir -p "$SCRIPT_DIR/output"  # Using -p to avoid issues if the directory exists

export TFD_HOME="$SCRIPT_DIR/downward"

python3 "$TFD_HOME/translate/translate.py" "$DOMAIN_FILE" "$PROBLEM_FILE"

# if ! (cd "$SCRIPT_DIR/output" && python3 "$TFD_HOME/translate/translate.py" "$DOMAIN_FILE" "$PROBLEM_FILE" > /dev/null 2>&1); then
#     echo "ERROR"
#     exit 1
# fi

# cd "$SCRIPT_DIR/output"

# "$TFD_HOME/preprocess/preprocess" < output.sas > /dev/null 2>&1
# "$TFD_HOME/search/search" y Y a T 10 t 5 e r O 1 C 1 p "$TFD_HOME/plan" < output > /dev/null 2>&1

# cat "$TFD_HOME/plan.1"
