# _stack_build_autocomplete()

# I am going to use COMPRELY that is properly from bash
_stack_build_autocomplete()
{
    #     TODO: My awesome autocomplete function, sugestion look how COMPREPLY works.
    
    # current typed
    local cur="${COMP_WORDS[COMP_CWORD]}"
    
    # Possible completions 
    local options="fail_detection_py fail_detection_cpp interfaces rosboard usr_msgs"

    # Generate suggestions
    COMPREPLY=($(compgen -W "$options" -- "$cur"))
}

alias stack-build-up-to="source /workspace/scripts/stack_build.sh --symlink-install --packages-up-to"
alias stack-build-select="source /workspace/scripts/stack_build.sh --symlink-install --packages-select"

complete -F _stack_build_autocomplete stack-build-up-to
complete -F _stack_build_autocomplete stack-build-select
