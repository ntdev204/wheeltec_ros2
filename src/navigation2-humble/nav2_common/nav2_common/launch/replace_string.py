
from typing import Dict
from typing import List
from typing import Text
import tempfile
import launch

class ReplaceString(launch.Substitution):

  def __init__(self,
    source_file: launch.SomeSubstitutionsType,
    replacements: Dict) -> None:
    super().__init__()

    from launch.utilities import normalize_to_list_of_substitutions
    self.__source_file = normalize_to_list_of_substitutions(source_file)
    self.__replacements = {}
    for key in replacements:
        self.__replacements[key] = normalize_to_list_of_substitutions(replacements[key])

  @property
  def name(self) -> List[launch.Substitution]:
    return self.__source_file

  def describe(self) -> Text:
    return ''

  def perform(self, context: launch.LaunchContext) -> Text:
    output_file = tempfile.NamedTemporaryFile(mode='w', delete=False)
    replacements = self.resolve_replacements(context)
    try:
      input_file = open(launch.utilities.perform_substitutions(context, self.name), 'r')
      self.replace(input_file, output_file, replacements)
    except Exception as err:
      print('ReplaceString substitution error: ', err)
    finally:
      input_file.close()
      output_file.close()
    return output_file.name

  def resolve_replacements(self, context):
    resolved_replacements = {}
    for key in self.__replacements:
      resolved_replacements[key] = launch.utilities.perform_substitutions(context, self.__replacements[key])
    return resolved_replacements

  def replace(self, input_file, output_file, replacements):
    for line in input_file:
      for key, value in replacements.items():
        if isinstance(key, str) and isinstance(value, str):
          if key in line:
            line = line.replace(key, value)
        else:
          raise TypeError('A provided replacement pair is not a string. Both key and value should be strings.')
      output_file.write(line)
