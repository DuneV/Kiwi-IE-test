_stack_build_autocomplete()
{
    # TODO: My awesome autocomplete function, sugestion look how COMPREPLY works.
    
}

alias stack-build-up-to="source /workspace/scripts/stack_build.sh --symlink-install --packages-up-to"
alias stack-build-select="source /workspace/scripts/stack_build.sh --symlink-install --packages-select"


# complete -F _stack_build_autocomplete stack-build-up-to
# complete -F _stack_build_autocomplete stack-build-select