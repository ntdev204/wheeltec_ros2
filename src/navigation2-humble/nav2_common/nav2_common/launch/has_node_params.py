
from typing import List
from typing import Text

import yaml
import launch

import sys

class HasNodeParams(launch.Substitution):

  def __init__(self,
    source_file: launch.SomeSubstitutionsType,
    node_name: Text) -> None:
    super().__init__()

    from launch.utilities import normalize_to_list_of_substitutions
    self.__source_file = normalize_to_list_of_substitutions(source_file)
    self.__node_name = node_name

  @property
  def name(self) -> List[launch.Substitution]:
    return self.__source_file

  def describe(self) -> Text:
    return ''

  def perform(self, context: launch.LaunchContext) -> Text:
    yaml_filename = launch.utilities.perform_substitutions(context, self.name)
    data = yaml.safe_load(open(yaml_filename, 'r'))

    if self.__node_name in data.keys():
        return "True"
    return "False"
